import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { useVectraStore } from "../store";
import { PARAM_LINES } from "../test/fixtures";
import type { ParamInfo } from "../types";
import ParamsEditor from "./ParamsEditor";

const AIRMODE: ParamInfo = {
  name: "MC_AIRMODE", short: "Multicopter air-mode", long: "The air-mode enables the mixer to increase the total thrust.",
  type: "enum", unit: null, min: null, max: null, default: 0,
  values: { "0": "Disabled", "1": "Roll/Pitch", "2": "Roll/Pitch/Yaw" }, group: "Mixer Output", reboot: false,
  source: "src/lib/mixer_module/params.c",
};

type Call = { url: string; method: string; body?: unknown };

function mockFetch(boardRead: number | { status: number; detail: string }): Call[] {
  const calls: Call[] = [];
  const ok = (data: unknown) =>
    Promise.resolve({ ok: true, status: 200, json: async () => data, text: async () => "" } as Response);
  const fail = (status: number, detail: string) =>
    Promise.resolve({ ok: false, status, json: async () => ({ detail }), text: async () => JSON.stringify({ detail }) } as Response);
  vi.stubGlobal(
    "fetch",
    vi.fn((input: RequestInfo | URL, init?: RequestInit) => {
      const url = String(input);
      const method = init?.method ?? "GET";
      const body = init?.body ? (JSON.parse(String(init.body)) as unknown) : undefined;
      calls.push({ url, method, body });
      if (url.startsWith("/api/px4/params?")) {
        const q = (new URL(url, "http://x").searchParams.get("q") ?? "").toLowerCase();
        return ok({ count: 715, results: q && "mc_airmode multicopter air-mode".includes(q) ? [AIRMODE] : [] });
      }
      if (url === "/api/px4/params/MC_AIRMODE") return ok(AIRMODE);
      if (url.startsWith("/api/px4/params/")) return fail(404, "not in the pinned tree's parameter definitions");
      if (url.startsWith("/api/board/param?") && method === "GET") {
        return typeof boardRead === "number"
          ? ok({ name: "MC_AIRMODE", value: boardRead, type_code: 6 })
          : fail(boardRead.status, boardRead.detail);
      }
      if (url === "/api/board/param" && method === "POST") {
        const b = body as { name: string; value: number };
        return ok({ name: b.name, wanted: b.value, before: 1, after: b.value, type_code: 6, verified: true, reboot_required: false });
      }
      return fail(404, `unmocked ${url}`);
    }),
  );
  return calls;
}

describe("ParamsEditor", () => {
  beforeEach(() => {
    act(() => {
      useVectraStore.getState().reset();
      useVectraStore.setState({ paramsLines: PARAM_LINES });
    });
  });
  afterEach(() => {
    useVectraStore.getState().reset();
    vi.unstubAllGlobals();
  });

  it("searches, picks a parameter, shows the board value and writes a new one", async () => {
    const calls = mockFetch(1);
    render(<ParamsEditor />);
    fireEvent.change(screen.getByRole("combobox", { name: "search PX4 parameters" }), { target: { value: "airmode" } });
    fireEvent.mouseDown(await screen.findByRole("option", { name: /MC_AIRMODE/ }));
    await waitFor(() => expect(screen.getByTestId("board-value")).toHaveTextContent("1"));
    expect(screen.getByText("Multicopter air-mode")).toBeInTheDocument();
    expect(screen.getByTestId("param-message")).toHaveTextContent("board has MC_AIRMODE = 1");
    const write = screen.getByRole("button", { name: "write" });
    expect(write).toBeDisabled(); // unchanged
    fireEvent.change(screen.getByLabelText("new"), { target: { value: "0" } });
    expect(write).toBeEnabled();
    fireEvent.click(write);
    await waitFor(() => expect(screen.getByTestId("param-message")).toHaveTextContent("MC_AIRMODE 1 -> 0 saved"));
    const post = calls.find((c) => c.url === "/api/board/param" && c.method === "POST");
    expect(post?.body).toEqual({ name: "MC_AIRMODE", value: 0, port: "auto" });
    expect(screen.getByTestId("board-value")).toHaveTextContent("0");
    // the params file preview is still there, folded
    expect(screen.getByLabelText("PX4 params preview")).toHaveValue(PARAM_LINES.join("\n"));
  });

  it("accepts a typed name outside the catalogue and shows the board's answer", async () => {
    mockFetch({ status: 503, detail: "the board has no parameter NOPE_X" });
    render(<ParamsEditor />);
    const box = screen.getByRole("combobox", { name: "search PX4 parameters" });
    fireEvent.change(box, { target: { value: "nope_x" } });
    await screen.findByRole("option", { name: /NOPE_X/ });
    fireEvent.keyDown(box, { key: "Enter" });
    await waitFor(() => expect(screen.getByTestId("param-message")).toHaveTextContent("no parameter NOPE_X"));
    expect(screen.getByText("not in the pinned tree's definitions")).toBeInTheDocument();
    expect(screen.getByTestId("board-value")).toHaveTextContent("unread");
    expect(screen.getByRole("button", { name: "write" })).toBeDisabled();
    fireEvent.click(screen.getByRole("button", { name: "clear" }));
    expect(screen.queryByTestId("param-card")).toBeNull();
  });
});
