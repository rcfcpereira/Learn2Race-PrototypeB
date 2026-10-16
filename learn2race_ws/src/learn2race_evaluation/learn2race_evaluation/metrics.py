"""Shared statistics and LaTeX emission for the evaluation campaign.

Every analysis in this package returns a Result. A Result knows how to print
itself for the terminal and how to render itself as a LaTeX table row, so the
numbers that go into the dissertation are the numbers the tools produced, with
no manual transcription step in between.
"""

# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: evaluation campaign
# More details: README.md of this package

from dataclasses import dataclass, field
from statistics import mean, median, pstdev
from typing import List, Optional


def summarise(values: List[float]) -> dict:
    """Return the summary statistics used throughout the campaign."""
    if not values:
        return {'n': 0}
    ordered = sorted(values)
    return {
        'n': len(values),
        'mean': mean(values),
        'median': median(values),
        'sd': pstdev(values) if len(values) > 1 else 0.0,
        'min': ordered[0],
        'max': ordered[-1],
        'p95': ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))],
    }


@dataclass
class Result:
    """One measured quantity, tied to the requirement it verifies."""

    requirement: str
    metric: str
    unit: str
    values: List[float] = field(default_factory=list)
    acceptance: Optional[str] = None
    passed: Optional[bool] = None
    note: str = ''

    @property
    def stats(self) -> dict:
        return summarise(self.values)

    def verdict(self) -> str:
        if self.passed is None:
            return 'not assessed'
        return 'pass' if self.passed else 'FAIL'

    def to_text(self) -> str:
        s = self.stats
        if s['n'] == 0:
            return f'{self.requirement:8s} {self.metric:32s} no samples'
        return (f'{self.requirement:8s} {self.metric:32s} '
                f"n={s['n']:4d}  mean={s['mean']:9.4f}  sd={s['sd']:8.4f}  "
                f"min={s['min']:9.4f}  max={s['max']:9.4f}  [{self.verdict()}]")

    def to_latex_row(self) -> str:
        """Render as a row for the results tables in Chapter 5."""
        s = self.stats
        if s['n'] == 0:
            return (f'\t\t\t\t\\ac{{{self.requirement.lower()}}} & {self.metric} & '
                    f'--- & --- & --- & not assessed \\\\')
        return (f'\t\t\t\t\\ac{{{self.requirement.lower()}}} & {self.metric} & '
                f"\\num{{{s['mean']:.4g}}} & \\num{{{s['sd']:.3g}}} & "
                f"{s['n']} & {self.verdict()} \\\\")


def latex_table(results: List[Result], caption: str, label: str) -> str:
    """Assemble a complete results table for the dissertation."""
    rows = '\n\t\t\t\t\\addlinespace\n'.join(r.to_latex_row() for r in results)
    return f"""\t\t\t\\begin{{table}}[hbt]
\t\t\t\t\\centering
\t\t\t\t\\small
\t\t\t\t\\caption{{{caption}}}
\t\t\t\t\\label{{{label}}}
\t\t\t\t\\begin{{tabularx}}{{\\linewidth}}{{@{{}}>{{\\raggedright\\arraybackslash}}p{{1.3cm}} X >{{\\raggedright\\arraybackslash}}p{{1.8cm}} >{{\\raggedright\\arraybackslash}}p{{1.6cm}} >{{\\raggedright\\arraybackslash}}p{{1.0cm}} >{{\\raggedright\\arraybackslash}}p{{1.6cm}}@{{}}}}
\t\t\t\t\t\\toprule
\t\t\t\t\t\\rowcolor[rgb]{{0.863,0.902,0.945}}
\t\t\t\t\t\\textbf{{Req.}} & \\textbf{{Metric}} & \\textbf{{Mean}} & \\textbf{{SD}} & \\textbf{{n}} & \\textbf{{Verdict}} \\\\
\t\t\t\t\t\\midrule
{rows}
\t\t\t\t\t\\bottomrule
\t\t\t\t\\end{{tabularx}}
\t\t\t\\end{{table}}
"""
