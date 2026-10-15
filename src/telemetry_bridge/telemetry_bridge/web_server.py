# Copyright 2026 Drone Waypoint Demo
"""Small HTTP and WebSocket API for the ground-station page."""

import asyncio
import json
from pathlib import Path

from aiohttp import web


ALLOWED_COMMANDS = frozenset(("start", "abort", "rtl", "reload"))


def create_app(store, commands, index_path):
    """Build the aiohttp app. store and commands are shared with the ROS node."""
    app = web.Application()
    app["store"] = store
    app["commands"] = commands
    app["index_path"] = Path(index_path)
    app["clients"] = set()
    app.router.add_get("/", _index)
    app.router.add_get("/api/health", _health)
    app.router.add_get("/api/state", _state)
    app.router.add_get("/api/mission", _mission)
    app.router.add_post("/api/command", _command)
    app.router.add_get("/ws", _websocket)
    app.on_startup.append(_on_startup)
    app.on_cleanup.append(_on_cleanup)
    return app


async def _on_startup(app):
    app["broadcaster"] = asyncio.create_task(_broadcast(app))


async def _on_cleanup(app):
    task = app.get("broadcaster")
    if task is not None:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    for client in list(app["clients"]):
        await client.close()
    app["clients"].clear()


async def _broadcast(app):
    try:
        while True:
            payload = json.dumps(app["store"].snapshot())
            stale = []
            for client in list(app["clients"]):
                try:
                    await client.send_str(payload)
                except (ConnectionError, RuntimeError):
                    stale.append(client)
            for client in stale:
                app["clients"].discard(client)
            await asyncio.sleep(0.2)
    except asyncio.CancelledError:
        raise


async def _index(request):
    path = request.app["index_path"]
    if not path.is_file():
        return web.Response(status=404, text="ground station UI is not installed")
    return web.FileResponse(path)


async def _health(request):
    return web.json_response(request.app["store"].health())


async def _state(request):
    return web.json_response(request.app["store"].snapshot())


async def _mission(request):
    snapshot = request.app["store"].snapshot()
    return web.json_response(
        {
            "mission": snapshot["mission"],
            "plan": snapshot["plan"],
        }
    )


async def _command(request):
    try:
        payload = await request.json()
    except json.JSONDecodeError:
        return web.json_response({"error": "expected JSON"}, status=400)
    if not isinstance(payload, dict):
        return web.json_response({"error": "expected an object"}, status=400)
    command = payload.get("command", "")
    if not isinstance(command, str):
        return web.json_response({"error": "command must be a string"}, status=400)
    normalized = command.strip().lower()
    if normalized not in ALLOWED_COMMANDS:
        return web.json_response(
            {"error": "unsupported command '%s'" % command},
            status=400,
        )
    request.app["commands"].push(normalized)
    return web.json_response({"accepted": True, "command": normalized}, status=202)


async def _websocket(request):
    socket = web.WebSocketResponse()
    await socket.prepare(request)
    request.app["clients"].add(socket)
    try:
        await socket.send_str(json.dumps(request.app["store"].snapshot()))
        async for _message in socket:
            pass
    finally:
        request.app["clients"].discard(socket)
    return socket
