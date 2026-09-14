import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { setRebootRecheckMs, useVectraStore } from "../store";
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
      if (url === "/api/board/thrust_feed/start" || url.startsWith("/api/board/thrust_feed?")) {
        return ok({
          running: true, port: "COM7", uptime_s: 1.2, error: null,
          latest: { rc_raw: 1200, rc_channels: [1500, 1500, 1200], throttle: 0.23, throttle_source: "MANUAL_CONTROL", thrust_sp: 0.35,
            armed: true, hil: false, mode: "STABILIZED", main_us: [1301, 1302], aux_us: [1400] },
          params: { MPC_THR_HOVER: 0.5, MPC_MANTHR_MIN: 0.08, RC3_MIN: 1000, RC3_MAX: 2000 }, counts: {},
          samples: [{ t: 0, rc_raw: 1200, throttle: 0.2, thrust_sp: 0.3, armed: true, main_us: [], aux_us: [] },
            { t: 1, rc_raw: 1200, throttle: 0.23, thrust_sp: 0.35, armed: true, main_us: [], aux_us: [] }],
        });
      }
      if (url === "/api/board/thrust_feed/stop") return ok({ stopped: true });
      if (url === "/api/board/pull_log") {
        return ok({
          port: "COM7", path: "exports/logs/20260913_1900_flight_log012.ulg", log_id: 12, num_logs: 15,
          size: 3_200_000, time_utc: 1_757_790_000, seconds: 64, url: "/api/board/logs/20260913_1900_flight_log012.ulg",
        });
      }
      if (url === "/api/board/flight") {
        return ok({
          ...pushed, sent: 30, changed: ["SYS_HITL", "SYS_AUTOSTART"], verified: 30,
          params: { SYS_HITL: 0, SYS_AUTOSTART: 4001, EKF2_EN: 1 },
          sources: { SYS_HITL: "hitl_undo", SYS_AUTOSTART: "backup", EKF2_EN: "backup" },
          warnings: ["controller gains stay"], base: "tests/fixtures/flight.params", reboot_required: true,
        });
      }
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
      useVectraStore.getState().reset();
      useVectraStore.setState({ scenario: tenFanScenario(), paramsLines: PARAM_LINES });
    });
  });
  afterEach(() => {
    useVectraStore.getState().reset();
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
    expect(useVectraStore.getState().board.message).toMatch(/no Pixhawk/);
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
    // the re-check after the reboot ran and the board reports again (it follows a timer, so wait)
    await waitFor(() => expect(calls.filter((c) => c.url.startsWith("/api/board/status")).length).toBe(2));
  });

  it("Test thrust starts the live feed, shows the pipeline and stops it", async () => {
    const calls = mockFetch(online, pushed);
    render(<BoardPanel />);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Test thrust" }));
    });
    expect(calls.find((c) => c.url === "/api/board/thrust_feed/start")?.body).toMatchObject({ port: "auto" });
    const feed = screen.getByTestId("thrust-feed");
    expect(feed).toHaveTextContent("ARMED / STABILIZED");
    expect(feed).toHaveTextContent("1200 us");
    expect(feed).toHaveTextContent("23%");
    expect(feed).toHaveTextContent("35%");
    expect(feed).toHaveTextContent("MAIN 1");
    expect(feed).toHaveTextContent("1400");
    expect(screen.getByRole("button", { name: "Check" })).toBeDisabled();
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Stop test" }));
    });
    expect(calls.some((c) => c.url === "/api/board/thrust_feed/stop")).toBe(true);
    expect(useVectraStore.getState().thrust.running).toBe(false);
    expect(screen.getByRole("button", { name: "Check" })).not.toBeDisabled();
  });

  it("Latest flight data pulls the newest log and offers the file", async () => {
    const calls = mockFetch(online, pushed);
    render(<BoardPanel />);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Latest flight data" }));
    });
    expect(calls.find((c) => c.url === "/api/board/pull_log")?.body).toMatchObject({ port: "auto" });
    const link = screen.getByRole("link", { name: "download .ulg" }) as HTMLAnchorElement;
    expect(link.getAttribute("href")).toBe("/api/board/logs/20260913_1900_flight_log012.ulg");
    expect(useVectraStore.getState().board.message).toMatch(/log 12 of 15 saved \(3\.2 MB in 64 s/);
  });

  it("HIL off restores the flight set through /api/board/flight, not a bare SYS_HITL write", async () => {
    setRebootRecheckMs(0);
    const calls = mockFetch({ ...online, sys_hitl: 1 }, pushed);
    render(<BoardPanel />);
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "Check" }));
    });
    expect(screen.getByText("on (HITL)")).toBeInTheDocument();
    await act(async () => {
      fireEvent.click(screen.getByRole("button", { name: "HIL off" }));
    });
    const flight = calls.find((c) => c.url === "/api/board/flight");
    expect(flight?.body).toMatchObject({ port: "auto", scenario: { fans: expect.any(Array) } });
    expect(calls.some((c) => c.url === "/api/board/param")).toBe(false);
    expect(screen.getByText("off")).toBeInTheDocument();
    expect(useVectraStore.getState().board.message).toMatch(/HITL off: 30 parameters written \(2 from flight.params, SYS_AUTOSTART 4001, EKF2_EN 1\)/);
    expect(useVectraStore.getState().board.message).toMatch(/controller gains stay/);
  });
});
