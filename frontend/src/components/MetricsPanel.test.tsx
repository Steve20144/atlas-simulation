import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { useTiltlabStore } from "../store";
import { PARAM_LINES, sampleMetrics } from "../test/fixtures";
import MetricsPanel from "./MetricsPanel";

describe("MetricsPanel", () => {
  beforeEach(() => {
    act(() => {
      useTiltlabStore.getState().reset();
      useTiltlabStore.setState({ metrics: sampleMetrics(), paramsLines: PARAM_LINES });
    });
  });

  afterEach(() => {
    useTiltlabStore.getState().reset();
    vi.unstubAllGlobals();
  });

  it("renders params preview lines and copies them", async () => {
    const writeText = vi.fn(() => Promise.resolve());
    vi.stubGlobal("navigator", { ...navigator, clipboard: { writeText } });
    render(<MetricsPanel />);
    const box = screen.getByLabelText("PX4 params preview") as HTMLTextAreaElement;
    expect(box.value).toBe(PARAM_LINES.join("\n"));
    fireEvent.click(screen.getByRole("button", { name: "copy" }));
    expect(writeText).toHaveBeenCalledWith(PARAM_LINES.join("\n"));
    expect(await screen.findByRole("button", { name: "copied" })).toBeInTheDocument();
  });

  it("shows the estimated banner, badges, and hides a group when unticked", () => {
    render(<MetricsPanel />);
    expect(screen.getByRole("status")).toHaveTextContent(/estimated/i);
    expect(screen.getAllByTestId("badge").length).toBeGreaterThanOrEqual(6);
    expect(screen.getByText("Coupling", { selector: "h3" })).toBeInTheDocument();
    fireEvent.click(screen.getByLabelText("Coupling"));
    expect(screen.queryByText("Coupling", { selector: "h3" })).toBeNull();
  });
});
