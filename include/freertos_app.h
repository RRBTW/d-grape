#pragma once

/**
 * freertos_app.h — точка входа в FreeRTOS приложение.
 * Вызывается из main.c перед osKernelStart().
 */
void freertos_app_init(void);