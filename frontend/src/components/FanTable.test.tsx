import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { useTiltlabStore } from "../store";
import { installFetchMock, tenFanScenario } from "../test/fixtures";
import FanTable from "./FanTable";

describe("FanTable", () => {
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

  it("renders 10 rows", () => {
    render(<FanTable />);
    expect(screen.getAllByTestId(/fan-row-/)).toHaveLength(10);
  });

  it("arrow keys nudge tilt by 1 degree and follow the mirror lock", () => {
    render(<FanTable />);
    const tilt0 = screen.getByLabelText("Tilt fan 0") as HTMLInputElement;
    fireEvent.keyDown(tilt0, { key: "ArrowUp" });
    let fans = useTiltlabStore.getState().scenario.fans;
    expect(fans[0].tilt_deg).toBe(31);
    expect(fans[1].tilt_deg).toBe(31);
    fireEvent.keyDown(screen.getByLabelText("Tilt fan 0"), { key: "ArrowDown" });
    fireEvent.keyDown(screen.getByLabelText("Tilt fan 0"), { key: "ArrowDown" });
    fans = useTiltlabStore.getState().scenario.fans;
    expect(fans[0].tilt_deg).toBe(29);
  });

  it("azimuth nudge wraps and mirrors to 360 - az", () => {
    render(<FanTable />);
    act(() => useTiltlabStore.getState().updateFan(0, { azimuth_deg: 359 }));
    fireEvent.keyDown(screen.getByLabelText("Azimuth fan 0"), { key: "ArrowUp" });
    const fans = useTiltlabStore.getState().scenario.fans;
    expect(fans[0].azimuth_deg).toBe(0);
    expect(fans[1].azimuth_deg).toBe(0);
  });

  it("typed tilt is clamped to 0..90 and the mirror checkbox toggles the lock", () => {
    render(<FanTable />);
    fireEvent.change(screen.getByLabelText("Tilt fan 2"), { target: { value: "120" } });
    expect(useTiltlabStore.getState().scenario.fans[2].tilt_deg).toBe(90);
    fireEvent.click(screen.getByLabelText("Mirror lock fan 2"));
    expect(useTiltlabStore.getState().mirrorLock[2]).toBe(false);
    expect(screen.getByLabelText("Mirror lock fan 8")).toBeDisabled();
  });

  it("explains that azimuth has no effect on a fan at tilt 0", () => {
    render(<FanTable />);
    expect(screen.queryByRole("note")).toBeNull();
    fireEvent.change(screen.getByLabelText("Azimuth fan 8"), { target: { value: "90" } });
    expect(screen.getByRole("note")).toHaveTextContent(/fan 8: azimuth 90 has no effect at tilt 0.*points right/);
    fireEvent.change(screen.getByLabelText("Tilt fan 8"), { target: { value: "30" } });
    expect(screen.queryByRole("note")).toBeNull();
  });
});
