import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { tiltAzimuthToFwdSide } from "../geometry";
import { useVectraStore } from "../store";
import { installFetchMock, tenFanScenario } from "../test/fixtures";
import FanTable from "./FanTable";

describe("FanTable", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    installFetchMock();
    useVectraStore.getState().reset();
    useVectraStore.getState().setScenario(tenFanScenario());
    useVectraStore.getState().setAngleMode("tilt_azimuth");
  });

  afterEach(() => {
    useVectraStore.getState().reset();
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
    let fans = useVectraStore.getState().scenario.fans;
    expect(fans[0].tilt_deg).toBe(31);
    expect(fans[1].tilt_deg).toBe(31);
    fireEvent.keyDown(screen.getByLabelText("Tilt fan 0"), { key: "ArrowDown" });
    fireEvent.keyDown(screen.getByLabelText("Tilt fan 0"), { key: "ArrowDown" });
    fans = useVectraStore.getState().scenario.fans;
    expect(fans[0].tilt_deg).toBe(29);
  });

  it("azimuth nudge wraps and mirrors to 360 - az", () => {
    render(<FanTable />);
    act(() => useVectraStore.getState().updateFan(0, { azimuth_deg: 359 }));
    fireEvent.keyDown(screen.getByLabelText("Azimuth fan 0"), { key: "ArrowUp" });
    const fans = useVectraStore.getState().scenario.fans;
    expect(fans[0].azimuth_deg).toBe(0);
    expect(fans[1].azimuth_deg).toBe(0);
  });

  it("typed tilt is clamped to 0..180 and the mirror checkbox toggles the lock", () => {
    render(<FanTable />);
    fireEvent.change(screen.getByLabelText("Tilt fan 2"), { target: { value: "200" } });
    expect(useVectraStore.getState().scenario.fans[2].tilt_deg).toBe(180);
    fireEvent.click(screen.getByLabelText("Mirror lock fan 2"));
    expect(useVectraStore.getState().mirrorLock[2]).toBe(false);
    expect(screen.getByLabelText("Mirror lock fan 8")).toBeDisabled();
  });

  it("forward and side tilt edit the same fan independently and follow the mirror lock", () => {
    useVectraStore.getState().setAngleMode("fwd_side");
    render(<FanTable />);
    // fan 8 is vertical: leaning it 20 degrees to the right needs no prior tilt
    fireEvent.change(screen.getByLabelText("Side tilt fan 8"), { target: { value: "20" } });
    let f8 = useVectraStore.getState().scenario.fans[8];
    expect(f8.tilt_deg).toBeCloseTo(20, 5);
    expect(f8.azimuth_deg).toBeCloseTo(90, 5);
    // adding a forward lean keeps the side lean
    fireEvent.change(screen.getByLabelText("Forward tilt fan 8"), { target: { value: "10" } });
    f8 = useVectraStore.getState().scenario.fans[8];
    expect((screen.getByLabelText("Side tilt fan 8") as HTMLInputElement).value).toBe("20");
    expect((screen.getByLabelText("Forward tilt fan 8") as HTMLInputElement).value).toBe("10");
    expect(f8.tilt_deg).toBeGreaterThan(20);
    // fan 0 (tilt 30, azimuth 90) reads as side 30, forward 0; its mirror partner leans left
    expect((screen.getByLabelText("Side tilt fan 0") as HTMLInputElement).value).toBe("30");
    expect((screen.getByLabelText("Forward tilt fan 0") as HTMLInputElement).value).toBe("0");
    expect((screen.getByLabelText("Side tilt fan 1") as HTMLInputElement).value).toBe("-30");
    fireEvent.keyDown(screen.getByLabelText("Forward tilt fan 0"), { key: "ArrowUp" });
    const fans = useVectraStore.getState().scenario.fans;
    expect(tiltAzimuthToFwdSide(fans[0].tilt_deg, fans[0].azimuth_deg).fwd).toBeCloseTo(1, 5);
    expect(tiltAzimuthToFwdSide(fans[1].tilt_deg, fans[1].azimuth_deg).fwd).toBeCloseTo(1, 5);
    expect(tiltAzimuthToFwdSide(fans[1].tilt_deg, fans[1].azimuth_deg).side).toBeCloseTo(-30, 5);
  });

  it("tilt accepts a negative value (opposite lean) and past horizontal", () => {
    render(<FanTable />);
    fireEvent.change(screen.getByLabelText("Tilt fan 8"), { target: { value: "-30" } });
    let f8 = useVectraStore.getState().scenario.fans[8];
    expect([f8.tilt_deg, f8.azimuth_deg]).toEqual([30, 180]);
    fireEvent.change(screen.getByLabelText("Tilt fan 9"), { target: { value: "120" } });
    f8 = useVectraStore.getState().scenario.fans[9];
    expect(f8.tilt_deg).toBe(120);
    act(() => useVectraStore.getState().setAngleMode("fwd_side"));
    expect(screen.getByRole("note")).toHaveTextContent(/tilt 120 az 0: past horizontal/);
    expect((screen.getByLabelText("Forward tilt fan 8") as HTMLInputElement).value).toBe("-30");
  });
  it("a minus sign being typed into forward tilt is kept until the digits arrive", () => {
    useVectraStore.getState().setAngleMode("fwd_side");
    render(<FanTable />);
    const fwd8 = screen.getByLabelText("Forward tilt fan 8") as HTMLInputElement;
    // a lone "-" reads back from a number input as "" and must not be turned into 0
    fireEvent.change(fwd8, { target: { value: "" } });
    expect(fwd8.value).toBe("");
    expect(useVectraStore.getState().scenario.fans[8].tilt_deg).toBe(0);
    fireEvent.change(fwd8, { target: { value: "-25" } });
    const f8 = useVectraStore.getState().scenario.fans[8];
    expect(fwd8.value).toBe("-25");
    expect([f8.tilt_deg, f8.azimuth_deg]).toEqual([25, 180]);
    fireEvent.blur(fwd8);
    expect(fwd8.value).toBe("-25");
  });
});
