
#ifndef WIFI_H
#define WIFI_H
#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>

struct struct_wifiInfo
{
    char ssid[48];
    uint8_t encryptionType;
    int32_t RSSI;
    int32_t channel;
};

static struct struct_wifiInfo *wifiInfo;
static uint8_t count_wifiInfo;

static const IPAddress AP_IP(192, 168, 4, 1);
static DNSServer dnsServer;

// extern bool wificonfig_loop;

// Function declarations;
static void wifi_networks_scan();
static void wifiAPbegin(const char *WIFI_AP_ID, const char *WIFI_AP_PWD);
static int selectChannelForAp();
static void wifi_Config();
static void waitForWifiToConnect(int maxRetries);
static void wifiAPstop();
// static void connectWifi();

static void wifi_networks_scan()
{
    WiFi.disconnect(true);
    Serial.println("Scanning for WiFi networks..");
    count_wifiInfo = WiFi.scanNetworks(false /* scan async */, true /* show hidden networks */);
    delete[] wifiInfo;
    wifiInfo = new struct_wifiInfo[count_wifiInfo];

    for (int i = 0; i < count_wifiInfo; i++)
    {
        String SSID;
        uint8_t *BSSID;

        memset(&wifiInfo[i], 0, sizeof(struct_wifiInfo));

        WiFi.getNetworkInfo(i, SSID, wifiInfo[i].encryptionType,
                            wifiInfo[i].RSSI, BSSID, wifiInfo[i].channel);

        SSID.toCharArray(wifiInfo[i].ssid, sizeof(wifiInfo[0].ssid));
    }
}

static void wifiAPbegin(const char *WIFI_AP_ID, const char *WIFI_AP_PWD)
{
    Serial.print("Starting WiFiManager: ");
    Serial.print("AP ID: ");
    Serial.print(WIFI_AP_ID);
    Serial.print(" Password: ");
    Serial.println(WIFI_AP_PWD);

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(WIFI_AP_ID, WIFI_AP_PWD, selectChannelForAp());

    // Ensure we don't poison the client DNS cache
    dnsServer.setTTL(0);
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", AP_IP); // 53 is port for DNS server
}

static void wifi_Config()
{

    // setup_webserver(); //? Call in different section like void setup()
    unsigned time_for_wifi_config = 60000;

    // 10 minutes timeout for wifi config
    unsigned long last_page_load = millis();
    while ((millis() - last_page_load) < time_for_wifi_config + 500)
    {
        dnsServer.processNextRequest();
        // server.handleClient();
        yield();
    }

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);

    dnsServer.stop();
    delay(100);

    // WiFi.begin(cfg::wlanssid, cfg::wlanpwd);
    // wificonfig_loop = false;
}

static void wifiAPstop()
{
    WiFi.softAPdisconnect(true);
    dnsServer.stop();
    delay(100);
}

static int selectChannelForAp()
{
    std::array<int, 14> channels_rssi;
    std::fill(channels_rssi.begin(), channels_rssi.end(), -100);

    for (unsigned i = 0; i < count_wifiInfo; i++)
    {
        if (wifiInfo[i].RSSI > channels_rssi[wifiInfo[i].channel])
        {
            channels_rssi[wifiInfo[i].channel] = wifiInfo[i].RSSI;
        }
    }

    if ((channels_rssi[1] < channels_rssi[6]) && (channels_rssi[1] < channels_rssi[11]))
    {
        return 1;
    }
    else if ((channels_rssi[6] < channels_rssi[1]) && (channels_rssi[6] < channels_rssi[11]))
    {
        return 6;
    }
    else
    {
        return 11;
    }
}

static void waitForWifiToConnect(int maxRetries)
{
    int retryCount = 0;
    while ((WiFi.status() != WL_CONNECTED) && (retryCount < maxRetries))
    {
        delay(500);
        Serial.print(".");
        ++retryCount;
    }
    Serial.println();
}

/*****************************************************************
 * WiFi auto connecting script                                   *
 *****************************************************************/
static void connectWifi(const char *WLANSSID, const char *WLANPWD)
{
    Serial.print("Connecting to ");
    Serial.println(WLANSSID);
#if defined(ESP8266)
    // Enforce Rx/Tx calibration
    system_phy_set_powerup_option(1);
    // 20dBM == 100mW == max tx power allowed in europe
    WiFi.setOutputPower(20.0f);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.setPhyMode(WIFI_PHY_MODE_11N);
    delay(100);
#endif
    if (WiFi.getAutoConnect())
    {
        WiFi.setAutoConnect(false);
    }
    if (!WiFi.getAutoReconnect())
    {
        WiFi.setAutoReconnect(true);
    }
    WiFi.mode(WIFI_STA);
    WiFi.hostname(WLANSSID);
    WiFi.begin(WLANPWD, WLANPWD); // Start WiFI

    Serial.print("Connecting to: ");
    Serial.println(WLANSSID);

    waitForWifiToConnect(40); // 20 second timeout to connect to wifi details form SPIFFS config.json file

    if (WiFi.status() != WL_CONNECTED)
    {
        // String fss(cfg::fs_ssid);
        // wifiConfig();
        if (WiFi.status() != WL_CONNECTED)
        {
            waitForWifiToConnect(20); // 10 second timeout to connect
        }
    }

    // Check again if WiFi is connected else setup webserver again on AP mode
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("WiFi connected, IP is: ");
        Serial.println(WiFi.localIP().toString());
        // last_signal_strength = WiFi.RSSI();

        if (MDNS.begin(WLANSSID))
        {
            MDNS.addService("http", "tcp", 80);
            MDNS.addServiceTxt("http", "tcp", "PATH", "/config");
        }
    }
    else
    {
        // Set up AP mode as in WifiConfig
        Serial.println("Failed to connect to a WiFi hotspot. Setting AP Mode and webserver");
        // WiFi.mode(WIFI_AP);
        // const IPAddress apIP(192, 168, 4, 1);
        // WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
        // WiFi.softAP(cfg::fs_ssid, cfg::fs_pwd, selectChannelForAp());
        wifiAPbegin(WLANSSID, WLANPWD);
    }
}

#endif
// // FROM OLD FIRMWARE

// // init_config();
// // init_display();
// // init_lcd();
// // setupNetworkTime();
// // connectWifi();
// // setup_webserver();
// // createLoggerConfigs();
