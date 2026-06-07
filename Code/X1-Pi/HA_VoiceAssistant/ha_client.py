import logging

import requests

logger = logging.getLogger(__name__)


class HAAssistClient:
    """Home Assistant Conversation API 客户端

    调用 HA 的 /api/conversation/process 接口，将用户文本发送给对话代理（LLM），
    返回语音回复文本。
    """

    def __init__(
        self,
        ha_url: str,
        token: str,
        agent_id: str,
        timeout: int = 30,
    ) -> None:
        self._url = f"{ha_url.rstrip('/')}/api/conversation/process"
        self._headers = {
            "Authorization": f"Bearer {token}",
            "Content-Type": "application/json",
        }
        self._agent_id = agent_id
        self._timeout = timeout
        logger.info(
            "HA 客户端已初始化 (url=%s, agent=%s)", self._url, agent_id
        )

    def process(self, text: str) -> str:
        """发送用户文本到 HA 对话代理，返回语音回复"""
        resp = requests.post(
            self._url,
            json={"text": text, "agent_id": self._agent_id},
            headers=self._headers,
            timeout=self._timeout,
        )
        resp.raise_for_status()
        data = resp.json()
        # 从 HA 响应结构中提取纯文本回复
        reply = data["response"]["speech"]["plain"]["speech"]
        logger.info("HA 回复 (%d 字符): %s", len(reply), reply[:80])
        return reply
