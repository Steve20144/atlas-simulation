import { useEffect, useRef, useState, type KeyboardEvent } from "react";
import { api } from "../api";
import type { ParamInfo } from "../types";

const NAME_RE = /^[A-Z][A-Z0-9_]{1,15}$/;
export const SEARCH_DEBOUNCE_MS = 120;

type Option = { name: string; short: string };

/** Searchable dropdown over the PX4 parameter catalogue (name or description, GET
 * /api/px4/params). A typed name that is not in the catalogue can still be picked, so
 * parameters of modules outside the sparse checkout stay reachable; the board decides. */
export default function ParamSearch({
  onPick,
  disabled = false,
}: {
  onPick: (name: string) => void;
  disabled?: boolean;
}) {
  const [q, setQ] = useState("");
  const [results, setResults] = useState<ParamInfo[]>([]);
  const [open, setOpen] = useState(false);
  const [active, setActive] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const seq = useRef(0);

  useEffect(() => {
    if (!open) return;
    const id = ++seq.current;
    const t = setTimeout(() => {
      api
        .paramSearch(q, 30)
        .then((r) => {
          if (seq.current !== id) return;
          setResults(r);
          setError(null);
          setActive(0);
        })
        .catch((e: Error) => {
          if (seq.current === id) setError(e.message);
        });
    }, SEARCH_DEBOUNCE_MS);
    return () => clearTimeout(t);
  }, [q, open]);

  const typed = q.trim().toUpperCase();
  const freeName = NAME_RE.test(typed) && !results.some((r) => r.name === typed) ? typed : null;
  const options: Option[] = [
    ...results.map((r) => ({ name: r.name, short: r.short })),
    ...(freeName ? [{ name: freeName, short: "not in the pinned tree's definitions; the board decides" }] : []),
  ];

  const pick = (name: string) => {
    onPick(name);
    setQ("");
    setOpen(false);
  };
  const onKey = (e: KeyboardEvent<HTMLInputElement>) => {
    if (e.key === "ArrowDown") {
      e.preventDefault();
      setOpen(true);
      setActive((a) => Math.min(a + 1, Math.max(options.length - 1, 0)));
    } else if (e.key === "ArrowUp") {
      e.preventDefault();
      setActive((a) => Math.max(a - 1, 0));
    } else if (e.key === "Enter" && options[active]) {
      e.preventDefault();
      pick(options[active].name);
    } else if (e.key === "Escape") {
      setOpen(false);
    }
  };

  return (
    <div className="relative">
      <input
        role="combobox"
        aria-label="search PX4 parameters"
        aria-expanded={open}
        aria-controls="param-search-list"
        aria-autocomplete="list"
        className="ui-input w-full font-mono"
        placeholder="search parameters: name or description"
        spellCheck={false}
        disabled={disabled}
        value={q}
        onChange={(e) => {
          setQ(e.target.value);
          setOpen(true);
        }}
        onFocus={() => setOpen(true)}
        onBlur={() => setTimeout(() => setOpen(false), 120)}
        onKeyDown={onKey}
      />
      {open && (
        <ul
          id="param-search-list"
          role="listbox"
          className="absolute z-20 mt-1 max-h-64 w-full overflow-auto rounded border border-slate-700 bg-slate-950 text-[11px] shadow-lg"
        >
          {error && <li className="px-2 py-1 text-rose-300">{error}</li>}
          {!error && options.length === 0 && (
            <li className="px-2 py-1 text-slate-500">
              {q ? "no match; type a full name like MC_AIRMODE" : "loading"}
            </li>
          )}
          {options.map((o, i) => (
            <li
              key={o.name}
              role="option"
              aria-selected={i === active}
              className={`cursor-pointer px-2 py-1 ${i === active ? "bg-slate-800" : ""}`}
              onMouseDown={(e) => {
                e.preventDefault();
                pick(o.name);
              }}
              onMouseEnter={() => setActive(i)}
            >
              <span className="font-mono text-slate-100">{o.name}</span>
              <span className="ml-2 text-slate-400">{o.short}</span>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
