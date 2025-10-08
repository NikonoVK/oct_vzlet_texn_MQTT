#include <WiFi.h>
#include <AsyncMqttClient.h>
#include "bright.h"

// Настройки WiFi
#define WIFI_SSID "RC-VZLET"
#define WIFI_PASSWORD "36-25-14-85-46"

// Настройки MQTT брокера
#define MQTT_HOST "gpl.iot.nau-ra.ru"
#define MQTT_PORT 1883
#define MQTT_USER "project/button"
#define MQTT_PSWD "67896a60faf82dffc301"

// Топик для подписки
#define TOPIC_SUBSCRIBE "devices/button/set"

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// Объявление функции sendMessage для использования в onMqttConnect
#ifdef ABSC
void sendMessage(const char *topic, const char *variable, int value, const char *message);
#endif

void connectToWifi()
{
  Serial.println("Подключение к WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Подключение к MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi подключен");
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi отключен");
    xTimerStop(mqttReconnectTimer, 0);
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

// Обработчик подключения к MQTT
void onMqttConnect(bool session)
{
  Serial.println("Подключен к MQTT брокеру");

#ifdef ABSC
  Serial.println("Режим отправителя");
#else
  // Подписываемся на топик devices/button/set
  mqttClient.subscribe(TOPIC_SUBSCRIBE, 1);
  Serial.print("Подписались на топик: ");
  Serial.println(TOPIC_SUBSCRIBE);
#endif
}

// Обработчик отключения от MQTT
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Отключен от MQTT");
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

#ifdef ABSC
// ================= РЕЖИМ ОТПРАВИТЕЛЯ =================

int messageCounter = 0;

// Реализация функции sendMessage
void sendMessage(const char *topic, const char *variable, int value, const char *message)
{
  // Создаем JSON строку с переданными значениями
  String payload = String("{\"");
  payload += variable;
  payload += "\":";
  payload += value;
  payload += ",\"message\":\"";
  payload += message;
  payload += "\"}";

  uint16_t packetIdPub = mqttClient.publish(topic, 0, false, payload.c_str());

  Serial.print("Публикация на ");
  Serial.print(topic);
  Serial.print(", packetId: ");
  Serial.println(packetIdPub);
  Serial.print("Полезная нагрузка: ");
  Serial.println(payload);
  Serial.print("Счетчик сообщений: ");
  Serial.println(messageCounter);
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Запуск MQTT отправителя данных...");
  pinMode(BR_BUT, INPUT_PULLUP);

  // Таймеры переподключения
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  // Настройка MQTT клиента
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  // В режиме отправителя не нужен обработчик сообщений

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PSWD);

  connectToWifi();
}

bool najatie = false;

void loop()
{
  if (digitalRead(BR_BUT) == 1 && najatie)
  {
    sendMessage("devices/button/set", "but1", 1, "Кнопка нажата");
    najatie = false;
  }
  else if (!najatie && digitalRead(BR_BUT) == 0)
  {
    sendMessage("devices/button/set", "but1", 0, "Кнопка отпущена");
    najatie = true;
  }
  delay(1000);
}

#else
// ================= РЕЖИМ ПОЛУЧАТЕЛЯ =================

// Обработчик входящих сообщений - получаем данные but1
void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  // Преобразуем payload в строку
  String message = "";
  for (size_t i = 0; i < len; i++)
  {
    message += (char)payload[i];
  }

  Serial.println("=== ПОЛУЧЕНЫ ДАННЫЕ ===");
  Serial.print("Топик: ");
  Serial.println(topic);
  Serial.print("Данные: ");
  Serial.println(message);

  // Ищем переменную but1 в JSON
  if (message.indexOf("but1") != -1)
  {
    // Извлекаем значение but1
    int startIndex = message.indexOf("but1") + 6; // "but1":
    int endIndex = message.indexOf(",", startIndex);
    if (endIndex == -1)
      endIndex = message.indexOf("}", startIndex);

    if (startIndex > 5 && endIndex > startIndex)
    {
      String but1Value = message.substring(startIndex, endIndex);
      Serial.print("Найдена переменная but1 = ");
      Serial.println(but1Value);

      // Здесь можно обработать значение but1
      // Например, управлять светодиодом или другими устройствами
      if (but1Value == "1")
      {
        digitalWrite(BR_D3, 1);
      }
      else
      {
        digitalWrite(BR_D3, 0);
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Запуск MQTT получателя данных...");
  pinMode(BR_D3, OUTPUT);

  // Таймеры переподключения
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  // Настройка MQTT клиента
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage); // Обработчик входящих сообщений

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PSWD);

  connectToWifi();
}

void loop()
{
  // Просто ждем входящие сообщения
  delay(1000);
}
#endif