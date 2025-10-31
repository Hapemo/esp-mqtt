#!/usr/bin/env python3
import sys
from pathlib import Path


def to_c_string_lines(text: str) -> str:
	# Split preserving logical lines; don't keep trailing newline because we add \n explicitly
	lines = text.splitlines()
	out = []
	for line in lines:
		# Escape backslashes and double quotes for valid C string literals
		esc = line.replace("\\", r"\\").replace('"', r'\"')
		out.append(f'"{esc}\\n"')
	# Ensure final newline in the generated C source for consistency
	return "\n".join(out) + "\n"


def main() -> int:
	if len(sys.argv) != 3:
		print("Usage: embed_html.py <input_html> <output_c>", file=sys.stderr)
		return 2

	inp = Path(sys.argv[1])
	outp = Path(sys.argv[2])

	try:
		text = inp.read_text(encoding="utf-8")
	except Exception as e:
		print(f"Failed to read {inp}: {e}", file=sys.stderr)
		return 1

	c_lines = to_c_string_lines(text)

	header = (
		f"/* Generated file - do not edit (configured from {inp.as_posix()}) */\n"
		"#include <stddef.h>\n"
		"const char html_page[] =\n"
	)

	out_src = header + c_lines + ";\n"

	outp.parent.mkdir(parents=True, exist_ok=True)
	try:
		# Normalize to LF to avoid extra CRs in the C literal content
		outp.write_text(out_src, encoding="utf-8", newline="\n")
	except Exception as e:
		print(f"Failed to write {outp}: {e}", file=sys.stderr)
		return 1

	return 0


if __name__ == "__main__":
	raise SystemExit(main())

