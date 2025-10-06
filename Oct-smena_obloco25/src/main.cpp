#include <WiFi.h>
#include "bright.h"
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

// Настройки WiFi сети
#define WIFI_SSID "Vouva"        // SSID WiFi сети для подключения
#define WIFI_PASSWORD "Vova2010" // Пароль WiFi сети

// Настройки MQTT брокера
#define MQTT_HOST "gpl.iot.nau-ra.ru"    // Адрес MQTT сервера
#define MQTT_PORT 1883                   // Порт MQTT сервера (обычно 1883 для незашифрованного соединения)
#define MQTT_USER "project/button"       // Имя пользователя для аутентификации на MQTT сервере
#define MQTT_PSWD "67896a60faf82dffc301" // Пароль для аутентификации на MQTT сервере

// Глобальные переменные
AsyncMqttClient mqttClient;       // Асинхронный MQTT клиент для работы с протоколом
TimerHandle_t mqttReconnectTimer; // Таймер для переподключения к MQTT при обрыве связи
TimerHandle_t wifiReconnectTimer; // Таймер для переподключения к WiFi при обрыве связи

// Переменные для управления отправкой сообщений
bool firstMessageSent = true;
unsigned long lastMessageTime = 0;
int messageCounter = 0;
bool connected = false;

// Функция подключения к WiFi сети
void connectToWifi()
{
  Serial.println("Подключение к WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// Функция подключения к MQTT брокеру
void connectToMqtt()
{
  Serial.println("Подключение к MQTT брокеру...");
  mqttClient.connect();
}

/// @brief Обработчик событий WiFi , вызывается автоматически при изменении состояния WiFi соединения
/// @param event код события WiFi
void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] событие: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    // Событие: получен IP адрес (успешное подключение к WiFi)
    Serial.println("Подключено к WiFi");
    Serial.println("IP адрес: ");
    Serial.println(WiFi.localIP());
    xTimerStop(wifiReconnectTimer, 0);
    connectToMqtt(); // После подключения к WiFi подключаемся к MQTT
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    // Событие: отключение от WiFi
    Serial.println("Отключение от WiFi");
    xTimerStop(mqttReconnectTimer, 0);  // Останавливаем таймер переподключения MQTT
    xTimerStart(wifiReconnectTimer, 0); // Запускаем таймер переподключения WiFi
    break;
  }
}

/// @brief Функция для отправки сообщения
/// @param value значение для отправки
void sendMessage(int value)
{
  // Создаем JSON строку с переданным значением
  String payload = String("{\"var1\":");
  payload += value;
  payload += "}";

  uint16_t packetIdPub = mqttClient.publish("devices/device1", 0, false, payload.c_str());

  Serial.print("Публикация на devices/device1, packetId: ");
  Serial.println(packetIdPub);
  Serial.print("Полезная нагрузка: ");
  Serial.println(payload);
  Serial.print("Счетчик сообщений: ");
  Serial.println(messageCounter);
}

/// @brief Обработчик успешного подключения к MQTT брокеру
/// @param sessionPresent флаг наличия предыдущей сессии
void onMqttConnect(bool sessionPresent)
{
  Serial.println("MQTT брокер подключён");
  Serial.print("Присутствующие: ");
  Serial.println(sessionPresent);
  xTimerStop(mqttReconnectTimer, 0);

  connected = true;
  // Отправляем первое сообщение сразу
}

/// @brief Обработчик отключения от MQTT брокера
/// @param reason причина отключения
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("MQTT брокер отключен");
  Serial.print("Причина отключения: ");
  Serial.println(static_cast<int>(reason));
  connected = false;
  // Если WiFi все еще подключен, запускаем таймер переподключения к MQTT
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/// @brief Обработчик подтверждения публикации сообщения
/// @param packetId идентификатор пакета
void onMqttPublish(uint16_t packetId)
{
  Serial.println("Подтвержденная публикация.");
  Serial.print("PacketId: ");
  Serial.println(packetId);
}

// Функция инициализации (вызывается один раз при старте устройства)
void setup()
{
  // Инициализация последовательного порта для отладки
  Serial.begin(115200);
  Serial.println();
  Serial.println("Запуск устройства...");
  pinMode(BR_A3, INPUT);
  // Создание таймеров для переподключения
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // Регистрация обработчика событий WiFi
  WiFi.onEvent(WiFiEvent);

  // Настройка обработчиков событий MQTT клиента
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);

  // Настройка параметров MQTT соединения
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setClientId("esp-dfsdfsdf");
  mqttClient.setCleanSession(true);
  mqttClient.setCredentials(MQTT_USER, MQTT_PSWD);

  // Начало подключения к WiFi
  connectToWifi();
}

void loop()
{
  static int sms = 0;
  // Отправляем второе сообщение через 5 секунд после первого
  if ((lastMessageTime + 1000 < millis()) && connected)
  {
    lastMessageTime = millis();
    sendMessage((analogRead(BR_A3) - 2500) * 100 / (4100 - 2500));
  }

  // Небольшая задержка для стабильности
  delay(100);

}
