# Contributing

Thanks for wanting to contribute to this repo! This guide covers the layout of the repo and how
to work on each part.

## Repo structure

```
urc-2023/
├── src/                  # ROS 2 workspace (colcon) - see docs/architecture.mdx
├── scripts/              # Build/launch/deploy shell scripts, wrapped by the Makefile
├── docs/                 # Documentation site content
├── .devcontainer/        # Dockerfile, compose file, and bashrc for the dev container
├── Makefile              # `make build` / `make launch` / etc.
├── pyproject.toml        # Ruff configuration
└── mypy.ini              # Mypy configuration
```

For what each ROS package under `src/` actually does, see
[docs/architecture.mdx](docs/architecture.mdx) (also published at
[docs.trickfirerobotics.com](https://docs.trickfirerobotics.com/urc-2023/architecture)).

### `docs/` - Documentation site

Go look at the [trickfire-docs documentation](https://docs.trickfirerobotics.com/trickfire-docs/)
for more info on writing/previewing pages.

## Dev setup

Development happens inside the Docker dev container defined in `.devcontainer/`. Open the repo
in VS Code and choose **Reopen in Container**. See [Getting Started](docs/getting-started.mdx)
for the full walkthrough.

The dev container installs the git pre-commit hooks automatically (`postCreateCommand` in
`devcontainer.json`). If you're working outside the dev container, run `make hooks` once after
cloning to register them yourself.

## Formatting

All formatters run automatically on save in VS Code. Install the recommended extensions when
prompted.

| Language      | Formatter                                                   |
| ------------- | ----------------------------------------------------------- |
| Python        | [Ruff](https://docs.astral.sh/ruff/) (`charliermarsh.ruff`) |
| Shell         | [shfmt](https://github.com/mvdan/sh) (`mkhl.shfmt`)         |
| Markdown/JSON | [Prettier](https://prettier.io/) (`esbenp.prettier-vscode`) |

Pre-commit hooks enforce formatting automatically at commit time. You can also run them manually:

```bash
pre-commit run
```

Or run everything directly:

```bash
make format
```

Configuration lives in `pyproject.toml` (`[tool.ruff]`) and `mypy.ini`. Type checking (`mypy`)
runs in CI but isn't part of the pre-commit hooks — run it manually with:

```bash
mypy --exclude /test/ --exclude setup\.py --exclude vision_opencv ./src
```
