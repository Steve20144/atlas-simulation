import { useState, type InputHTMLAttributes } from "react";

type Props = Omit<InputHTMLAttributes<HTMLInputElement>, "value" | "onChange" | "type"> & {
  value: number;
  onCommit: (v: number) => void;
};

/**
 * Controlled number input that keeps what the user is typing while it is not yet a number (a
 * lone "-", "-0", an empty field). Without this a typed minus sign reads back as 0 and is wiped
 * before the digits arrive. Commits every finite value; resyncs to the stored value on blur or
 * on an arrow-key nudge.
 */
export default function SignedNumberInput({ value, onCommit, onBlur, onKeyDown, ...rest }: Props) {
  const [draft, setDraft] = useState<string | null>(null);
  return (
    <input {...rest} type="number" value={draft ?? value}
      onChange={(e) => {
        const text = e.target.value;
        setDraft(text);
        const n = Number(text);
        if (text !== "" && Number.isFinite(n)) onCommit(n);
      }}
      onBlur={(e) => { setDraft(null); onBlur?.(e); }}
      onKeyDown={(e) => {
        if (e.key === "ArrowUp" || e.key === "ArrowDown") setDraft(null);
        onKeyDown?.(e);
      }} />
  );
}
