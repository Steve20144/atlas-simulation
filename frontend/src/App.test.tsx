import { render, screen } from "@testing-library/react";
import App, { APP_NAME } from "./App";
import { emptyScenario, useScenarioStore } from "./store";

describe("App shell", () => {
  it("renders the app name", () => {
    render(<App />);
    expect(screen.getByRole("heading", { level: 1 })).toHaveTextContent(APP_NAME);
  });

  it("shows the scenario name from the store", () => {
    const base = emptyScenario();
    useScenarioStore.getState().setScenario({ ...base, meta: { ...base.meta, name: "demo" } });
    render(<App />);
    expect(screen.getByTestId("scenario-name")).toHaveTextContent("demo");
    useScenarioStore.getState().reset();
  });
});
