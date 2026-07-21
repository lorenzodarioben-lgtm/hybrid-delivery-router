FROM python:3.12-slim
WORKDIR /app
COPY . .
RUN pip install --no-cache-dir . && useradd --create-home appuser
USER appuser
EXPOSE 8000
HEALTHCHECK CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/health')"
CMD ["python", "-m", "uvicorn", "hybrid_delivery_router.api:create_app", "--factory", "--host", "0.0.0.0"]
