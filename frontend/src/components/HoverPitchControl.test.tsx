import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { useTiltlabStore } from "../store";
import { installFetchMock, tenFanScenario } from "../test/fixtures";
import HoverPitchControl from "./HoverPitchControl";

describe("HoverPitchControl", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    installFetchMock();
    useTiltlabStore.getState().reset();
    useTiltlabStore.getState().setScenario(tenFanScenario());
  });

  afterEach(() => {
    useTiltlabStore.getState().reset();
    vi.unstubAllGlobals();
    vi.useRealTimers();
  });

  it("shows 0 when the scenario has no hover pitch and writes the typed value into the frame", () => {
    render(<HoverPitchControl />);
    const input = screen.getByLabelText("Hover pitch") as HTMLInputElement;
    expect(input.value).toBe("0");
    fireEvent.change(input, { target: { value: "25" } });
    expect(useTiltlabStore.getState().scenario.frame.hover_pitch_deg).toBe(25);
    expect(input.value).toBe("25");
  });

  it("quick buttons set the value and the active one is highlighted", () => {
    render(<HoverPitchControl />);
    fireEvent.click(screen.getByRole("button", { name: "15" }));
    expect(useTiltlabStore.getState().scenario.frame.hover_pitch_deg).toBe(15);
    expect(screen.getByRole("button", { name: "15" }).className).toContain("border-sky-400");
    expect(screen.getByRole("button", { name: "0" }).className).not.toContain("border-sky-400");
  });

  it("clamps to the backend bounds and triggers a metrics refresh", async () => {
    const { calls } = installFetchMock();
    render(<HoverPitchControl />);
    fireEvent.change(screen.getByLabelText("Hover pitch"), { target: { value: "120" } });
    expect(useTiltlabStore.getState().scenario.frame.hover_pitch_deg).toBe(90);
    await act(async () => {
      await vi.runAllTimersAsync();
    });
    const metrics = calls.filter((c) => c.url === "/api/metrics");
    expect(metrics.length).toBeGreaterThan(0);
    const sent = metrics[metrics.length - 1].body as { scenario: { frame: { hover_pitch_deg: number } } };
    expect(sent.scenario.frame.hover_pitch_deg).toBe(90);
  });
});
