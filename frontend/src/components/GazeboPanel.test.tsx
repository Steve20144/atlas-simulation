import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { useTiltlabStore } from "../store";
import { installFetchMock, tenFanScenario } from "../test/fixtures";
import type { GazeboStatus } from "../types";
import GazeboPanel from "./GazeboPanel";

const status = (over: Partial<GazeboStatus>): GazeboStatus => ({
  available: true, running: false, mode: null, harness: null, command: null, log: null, tail: [],
  returncode: null, ...over,
});

describe("GazeboPanel", () => {
  beforeEach(() => {
    installFetchMock();
    useTiltlabStore.getState().reset();
    useTiltlabStore.getState().setScenario(tenFanScenario());
  });

  afterEach(() => {
    useTiltlabStore.getState().reset();
    vi.unstubAllGlobals();
  });

  it("posts the scenario and mode, then shows the running session and enables Stop", async () => {
    const running = status({ running: true, mode: "sitl", log: "exports/logs/gazebo_sitl.log",
      tail: ["Ready for takeoff!"] });
    const fetchMock = vi.fn((input: RequestInfo | URL, init?: RequestInit) => {
      const url = String(input);
      const body = init?.body ? JSON.parse(String(init.body)) : undefined;
      if (url === "/api/gazebo/launch") {
        expect(body.mode).toBe("sitl");
        expect(body.scenario.fans).toHaveLength(10);
        return Promise.resolve({ ok: true, status: 200, json: async () => running } as Response);
      }
      if (url === "/api/gazebo/stop") {
        return Promise.resolve({ ok: true, status: 200,
          json: async () => status({ stopped: true }) } as Response);
      }
      return Promise.resolve({ ok: true, status: 200, json: async () => ({}) } as Response);
    });
    vi.stubGlobal("fetch", fetchMock);

    render(<GazeboPanel />);
    expect(screen.getByText("idle")).toBeInTheDocument();
    await act(async () => {
      fireEvent.click(screen.getByText("SITL"));
    });
    expect(fetchMock).toHaveBeenCalledWith("/api/gazebo/launch", expect.anything());
    expect(screen.getByText("SITL running")).toBeInTheDocument();
    expect(screen.getByText(/Ready for takeoff!/)).toBeInTheDocument();
    expect((screen.getByText("HITL") as HTMLButtonElement).disabled).toBe(true);

    await act(async () => {
      fireEvent.click(screen.getByText("Stop"));
    });
    expect(fetchMock).toHaveBeenCalledWith("/api/gazebo/stop", expect.anything());
    expect(screen.getByText("idle")).toBeInTheDocument();
    expect(useTiltlabStore.getState().gazebo.message).toBe("stopped");
  });

  it("shows the backend's error when the launch is refused", async () => {
    vi.stubGlobal("fetch", vi.fn(() => Promise.resolve({ ok: false, status: 501,
      text: async () => "Gazebo launch needs wsl.exe on this host", json: async () => ({}) } as Response)));
    render(<GazeboPanel />);
    await act(async () => {
      fireEvent.click(screen.getByText("HITL"));
    });
    expect(screen.getByText(/needs wsl.exe/)).toBeInTheDocument();
    expect(screen.getByText("idle")).toBeInTheDocument();
  });
});
