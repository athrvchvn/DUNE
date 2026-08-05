# Design documents

Long-form background, kept because they explain *why* rather than *what*. They predate this
repository's restructure and use the project's earlier name (UWB RTLS), so where they
disagree with `docs/` or with the code, the code wins.

| Document | For |
|---|---|
| [`uwb_rtls_design.pdf`](uwb_rtls_design.pdf) ([`.tex`](uwb_rtls_design.tex)) | The full mathematical design: DS-TWR estimator derivation, multilateration, filtering, error budgets. |
| [`uwb_rtls_for_dummies.pdf`](uwb_rtls_for_dummies.pdf) ([`.tex`](uwb_rtls_for_dummies.tex)) | The same ground from scratch, assuming no UWB background. Start here if time-of-flight ranging is new to you. |

Known drift: they describe `matlab/+rtls` (now `matlab/+dune`), configuration under
`matlab/config/` (now `config/`), and a single-tag system without the rigid-body estimator
or the dual-tag link.

For current material see [../README.md](../README.md); the mathematics of the deployed
estimators is in [../estimators.md](../estimators.md).
