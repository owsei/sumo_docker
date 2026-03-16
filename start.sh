#!/bin/bash
# Inicia FastAPI en segundo plano
# uvicorn main:app --host 0.0.0.0 --port 8000 &
service nginx start 
fastapi run /code/app/main.py --port 8001
# Inicia Nginx en primer plano (esto mantiene el contenedor vivo)

# nginx -g "daemon off;"