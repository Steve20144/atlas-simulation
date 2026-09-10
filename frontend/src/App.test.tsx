import { render, screen, waitFor } from "@testing-library/react";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import App, { APP_NAME } from "./App";
import { useTiltlabStore } from "./store";
import { installFetchMock } from "./test/fixtures";

// jsdom has no WebGL; the 3D view is covered by the build, not by unit tests.
vi.mock("./components/Viewer3D", () => ({ default: () => <section data-testid="viewer3d" /> }));

describe("App shell", () => {
  beforeEach(() => {
    installFetchMock();
    useTiltlabStore.getState().reset();
  });

  afterEach(() => {
    useTiltlabStore.getState().reset();
    vi.unstubAllGlobals();
  });

  it("renders the three panels and loads the scenario list", async () => {
    render(<App />);
    expect(screen.getByRole("heading", { level: 1 })).toHaveTextContent(APP_NAME);
    expect(screen.getByTestId("viewer3d")).toBeInTheDocument();
    expect(screen.getByText("Fans")).toBeInTheDocument();
    expect(screen.getByText("Metrics")).toBeInTheDocument();
    await waitFor(() => expect(useTiltlabStore.getState().scenarioNames).toContain("baseline_dihedral30"));
    expect(screen.getByRole("option", { name: "baseline_dihedral30" })).toBeInTheDocument();
  });
});
