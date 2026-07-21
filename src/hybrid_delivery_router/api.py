"""FastAPI service factory for routing scenarios."""
from fastapi import FastAPI
from .scenarios import available_scenarios

def create_app() -> FastAPI:
    app = FastAPI(title="Hybrid Delivery Router")
    @app.get("/health")
    def health() -> dict[str, str]: return {"status": "ok"}
    @app.get("/api/v1/scenarios")
    def scenarios() -> dict[str, tuple[str, ...]]: return {"scenarios": available_scenarios()}
    return app
