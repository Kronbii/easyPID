# Contributing to easyPID

Thanks for your interest in improving `easyPID`.

## Ways To Contribute

- Report bugs
- Propose new features or improvements
- Improve documentation and examples
- Submit pull requests for fixes and enhancements

## Before You Start

- Check existing issues and pull requests to avoid duplicate work.
- For larger changes, open an issue first to align on scope and approach.

## Reporting Bugs

When opening a bug report, include:

- A clear description of the problem
- Steps to reproduce
- Expected behavior vs actual behavior
- Arduino board/core information
- Library version or commit SHA
- A minimal sketch that reproduces the issue

## Development Setup

1. Fork and clone the repository.
2. Install Arduino IDE or Arduino CLI.
3. Install this library locally for testing from your clone.
4. Build/compile affected examples for at least one representative board.

Optional local linting:

- Use `arduino-lint` if installed locally.
- The repository also validates pull requests with GitHub Actions.

## Coding Guidelines

- Keep changes focused and minimal.
- Preserve backward compatibility unless a breaking change is explicitly agreed.
- Prefer readable, maintainable code over clever shortcuts.
- Update comments/docs/examples when behavior or APIs change.

## Pull Request Checklist

Before opening a pull request, please confirm:

- The change is scoped to a single concern.
- Relevant examples compile after your change.
- New behavior is documented (README, docs, or inline comments as needed).
- Existing behavior is not silently broken.

## Commit Messages

- Use clear, descriptive commit titles.
- Reference issue numbers when applicable.

## Code of Conduct

By participating, you agree to follow our code of conduct:

- `CODE_OF_CONDUCT.md`
