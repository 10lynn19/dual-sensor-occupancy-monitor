#pragma once

/**
 * 手机热点场景（推荐）：
 *   STASSID / STAPSK = 手机热点名称与密码
 *   FUSION_HOST = 笔记本连上热点后的 IPv4（在 macOS「网络」或 Windows ipconfig 里看）
 *   常见示例：192.168.43.x（Android）、172.20.10.x（iPhone 热点），以你电脑为准。
 *   iPhone：设置 → 个人热点里显示的「无线局域网密码」上方名称，必须与 STASSID 完全一致。
 *   Mac 若把该热点显示为「Hidden Network」，ESP 仍可用正确 SSID+密码连接；固件已用全信道扫描
 *   以提高搜到热点的概率。
 */

/** 手机热点名称（与笔记本、ESP 连接同一热点） */
#define STASSID "MyPhoneHotspot"

/** 热点密码 */
#define STAPSK "your_hotspot_password"

/**
 * 运行 Python 融合服务的笔记本 IP（与 ESP 在同一网段）。
 * 手机开热点后，只填「电脑」的 IP，不要填手机网关 IP。
 */
#define FUSION_HOST "192.168.43.10"

/** 与 web/server.py 中 FUSION_TCP_PORT 一致 */
#define FUSION_TCP_PORT 8766
