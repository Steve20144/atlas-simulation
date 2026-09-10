import { useScenarioStore } from "./store";

export const APP_NAME = "tiltlab";

export default function App() {
  const scenarioName = useScenarioStore((s) => s.scenario.meta.name);
  return (
    <main className="flex min-h-screen flex-col items-center justify-center gap-2 p-8">
      <h1 className="text-3xl font-semibold tracking-tight">{APP_NAME}</h1>
      <p className="text-sm text-slate-400">
        Scenario: <span data-testid="scenario-name">{scenarioName}</span>
      </p>
    </main>
  );
}
