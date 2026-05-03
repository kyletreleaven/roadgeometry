# Backend

FastAPI server for the road matching web app.

## Run

```
uv run uvicorn app:app --reload
```

Runs on `http://localhost:8000`. The Vite dev server at `localhost:5173` proxies
`/status`, `/pins`, `/reset`, and `/state` to this port.
