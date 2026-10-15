# Copyright 2026 Drone Waypoint Demo
"""HTTP and WebSocket behavior of the ground-station API."""

import asyncio
import socket
from pathlib import Path

import aiohttp
from aiohttp import web
import pytest

from telemetry_bridge.state_store import CommandQueue, StateStore
from telemetry_bridge.web_server import create_app

_PAGE = Path(__file__).resolve().parents[1] / "static" / "index.html"


class _Server:
    def __init__(self, store, commands, index_path):
        self.store = store
        self.commands = commands
        self.index_path = index_path
        self.port = None

    async def __aenter__(self):
        app = create_app(self.store, self.commands, self.index_path)
        self._runner = web.AppRunner(app)
        await self._runner.setup()
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.bind(("127.0.0.1", 0))
        self.port = probe.getsockname()[1]
        probe.close()
        self._site = web.TCPSite(self._runner, "127.0.0.1", self.port)
        await self._site.start()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self._runner.cleanup()


def test_static_page_describes_the_demo_honestly():
    text = _PAGE.read_text(encoding="utf-8")
    assert "Kinematic desktop demo" in text
    assert "/api/command" in text
    assert "/ws" in text


def test_health_state_command_and_websocket(tmp_path):
    index = tmp_path / "index.html"
    index.write_text(_PAGE.read_text(encoding="utf-8"), encoding="utf-8")
    asyncio.run(_exercise(index))


async def _exercise(index):
    store = StateStore()
    commands = CommandQueue()
    async with _Server(store, commands, index) as server:
        base = "http://127.0.0.1:%d" % server.port
        async with aiohttp.ClientSession() as session:
            async with session.get(base + "/api/health") as response:
                assert response.status == 200
                body = await response.json()
                assert body["service"] == "telemetry_bridge"
                assert body["have_drone"] is False
            async with session.get(base + "/") as response:
                assert response.status == 200
                page = await response.text()
                assert "Kinematic desktop demo" in page
            async with session.post(
                base + "/api/command",
                json={"command": " Start "},
            ) as response:
                assert response.status == 202
                assert (await response.json())["command"] == "start"
            async with session.post(
                base + "/api/command",
                data=b"nope",
                headers={"Content-Type": "application/json"},
            ) as response:
                assert response.status == 400
            async with session.post(
                base + "/api/command",
                json={"command": "launch"},
            ) as response:
                assert response.status == 400
            assert commands.drain() == ["start"]
            store.update_drone(
                {
                    "mode": "FLYING",
                    "z": 6.5,
                    "battery_percent": 88.0,
                    "geofence_radius": 40.0,
                }
            )
            async with session.ws_connect(base + "/ws") as socket:
                message = await socket.receive_json()
                assert message["drone"]["mode"] == "FLYING"
                assert message["drone"]["z"] == pytest.approx(6.5)
            async with session.get(base + "/api/mission") as response:
                mission = await response.json()
                assert "plan" in mission
                assert "waypoints" in mission["plan"]
