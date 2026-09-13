import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { setRebootRecheckMs, useTiltlabStore } from "../store";
import { PARAM_LINES, tenFanScenario } from "../test/fixtures";
import type { BoardPushResult, BoardStatus } from "../types";
import BoardPanel from "./BoardPanel";
import BoardPill from "./BoardPill";

const offline: BoardStatus = {
  connected: false, port: null, system_id: null, firmware: null, board_id: null, armed: null, hil: null,
  mode: null, sys_hitl: null, ports: [], message: "no Pixhawk on any serial port",
};
const online: BoardStatus = {
  ...offline, connected: true, port: "COM7", firmware: "1.17.0", armed: false, hil: true, mode: "MANUAL", sys_hitl: 0,
  ports: [{ device: "COM7", description: "PX4 FMU v6X.x", vid: 0x3185, pid: 0x0035, pixhawk: true }],
  message: "autopilot heartbeat received",
};
const pushed: BoardPushResult = {
  port: "COM7", sent: 86, changed: ["CA_ROTOR_COUNT"], verified: 86, mismatches: [],
  backup: "exports/board/20260912_1200_fixture10_before.params", console: "",
};

function mockFetch(status: BoardStatus, push: BoardPushResult | { status: number; detail: string }) {
  const calls: { url: string; body?: unknown }[] = [];
  vi.stubGlobal(
    "fetch",
    vi.fn((input: RequestInfo | URL, init?: RequestInit) => {
      const url = String(input);
      calls.push({ url, body: init?.body ? JSON.parse(String(init.body)) : undefined });
      const ok = (data: unknown) =>
        Promise.resolve({ ok: true, status: 200, json: async () => data, text: async () => "" } as Response);
      if (url.startsWith("/api/board/status")) return ok(status);
      if (url === "/api/board/param") {
        const b = JSON.parse(String(init?.body)) as { name: string; value: number };
        return ok({ name: b.name, wanted: b.value, before: 0, after: b.value, type_code: 6, verified: true, reboot_required: true });
      }
      if (url === "/api/board/reboot") return ok({ port: "COM7", rebooted: true });
      if (url === "/api/board/push") {
        if ("detail" in push) {
          return Promise.resolve({ ok: false, status: push.status, text: async () => push.detail } as Response);
        }
        return ok(push);
      }
      return Promise.resolve({ ok: false, status: 404, text: async () => "" } as Response);
    }),
  );
  return calls;
}

describe("BoardPanel and BoardPill", () => {
  beforeEach(() => {
    act(() => {
      useTiltlabStore.getState().reset();
      useTiltlabStore.setState({ scenario: tenFanScenario(), paramsLines: PARAM_LINES });
    });
  });
  afterEach(() => {
    useTiltlabStore.getState().reset();
    vi.unstubAllGlobals();
  });

  it("reports offline after a check and shows the message", async () => {
    mockFetch(offline, pushed);
    render(<BoardPill />);
    expect(screen.getByTestId("board-pill")).toHaveAttribute("data-state", "unknown");
    await act(async () => {
      fireEvent.click(screen.getByTestId("board-pill"));
    });
    expect(screen.getByTestId("board-pill")).toHaveAttribute("data-state", "offline");
    expect(useTiltlabStore.getState().board.message).toMatch(/no Pixhawk/);
  });

  it("checks, lists the port, then uploads after a confirming second click", async () => {
    const calls = mockFetch(online, pushed);
    render(<BoardPanel />);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Check" }));
    });
    expect(screen.getByText("connected")).toBeInTheDocument();
    expect(screen.getByRole("option", { name: "COM7 (Pixhawk)" })).toBeInTheDocument();
    expect(screen.getByText("1.17.0")).toBeInTheDocument();

    fireEvent.click(screen.getByRole("button", { name: "Upload params" }));
    const confirm = screen.getByRole("button", { name: `confirm: write ${PARAM_LINES.length} params` });
    expect(calls.some((c) => c.url === "/api/board/push")).toBe(false);
    await act(async () => {
      fireEvent.click(confirm);
    });
    const push = calls.find((c) => c.url === "/api/board/push");
    expect(push?.body).toMatchObject({ concept: "stock", port: "auto" });
    expect(screen.getByText(/86 of 86 parameters verified on COM7/)).toBeInTheDocument();
    expect(screen.getByText(/fixture10_before.params/)).toBeInTheDocument();
  });

  it("surfaces the backend error when no board is plugged in", async () => {
    mockFetch(offline, { status: 404, detail: "no Pixhawk on any serial port" });
    render(<BoardPanel />);
    fireEvent.click(screen.getByRole("button", { name: "Upload params" }));
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: /confirm: write/ }));
    });
    expect(screen.getByText(/failed \(404\) no Pixhawk/)).toBeInTheDocument();
  });

  it("toggles SYS_HITL and reboots after a confirming click", async () => {
    setRebootRecheckMs(0);
    const calls = mockFetch(online, pushed);
    render(<BoardPanel />);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Check" }));
    });
    expect(screen.getByText("off")).toBeInTheDocument();
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "HIL on" }));
    });
    expect(calls.find((c) => c.url === "/api/board/param")?.body).toMatchObject({ name: "SYS_HITL", value: 1 });
    expect(screen.getByText("on (HITL)")).toBeInTheDocument();
    expect(screen.getByText(/reboot the board for it to take effect/)).toBeInTheDocument();

    fireEvent.click(screen.getByRole("button", { name: "Reboot board" }));
    expect(calls.some((c) => c.url === "/api/board/reboot")).toBe(false);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "confirm reboot" }));
    });
    expect(calls.some((c) => c.url === "/api/board/reboot")).toBe(true);
    // the re-check after the reboot ran and the board reports again
    expect(calls.filter((c) => c.url.startsWith("/api/board/status")).length).toBe(2);
  });
});
