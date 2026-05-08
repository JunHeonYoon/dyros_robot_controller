# Documentation Workflow

## Local Preview

Install dependencies:

```bash
python3 -m pip install -r docs/requirements.txt
```

Serve locally:

```bash
mkdocs serve
```

Build strictly before publishing:

```bash
mkdocs build --strict
```

## Writing Rules

- Keep user-facing guides in `docs/`.
- Keep command examples copy-pasteable from the repository root unless the page says otherwise.
- Prefer short task-focused pages over one large reference page.
- Link to source files when a concept depends on implementation details.

## Deployment

The GitHub Actions workflow in `.github/workflows/docs.yml` builds the MkDocs site and publishes the generated `site/` directory through GitHub Pages.

GitHub Pages should be configured to use **GitHub Actions** as its source.
