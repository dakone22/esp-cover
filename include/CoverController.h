#ifndef COVERCONTROLLER_H
#define COVERCONTROLLER_H

#include "cover.h"

/**
 * @brief Интерфейс, используемый обработчиком MQTT-запросов для.
 */
class ICoverController {
public:
    /**
     * @brief Перечисление, представляющее состояния, которые приходят из MQTT-запросов
     */
    enum State {
        OPEN,   /**< Двигаться в сторону открытия. */
        CLOSE,  /**< Двигаться в сторону закрытия. */
        STOP    /**< Остановка движения. */
    };

    /**
     * @brief Обработка команды установки целевой позиции шторы.
     * @param position Целевая позиция для установки.
     */
    virtual void onSetPosition(int position) = 0;

    /**
     * @brief Обработка команды установки состояния шторы.
     * @param state Состояние управления шторой, которое необходимо установить.
     */
    virtual void onSet(State state) = 0;

    virtual ~ICoverController() = default;
};

/**
 * @brief Конкретная реализация интерфейса ICoverController.
 */
class CoverController : public ICoverController {
private:
    std::shared_ptr<IPositionCover> _cover;

public:
    /**
     * @brief Конструктор для класса CoverController.
     * @param cover Общий указатель на экземпляр шторы.
     */
    explicit CoverController(std::shared_ptr<IPositionCover> cover);

    inline void onSetPosition(int position) override { _cover->setTargetPosition(position); };

    inline void onSet(State state) override {
        if (state == STOP) {
            _cover->stop();
        } else if (state == OPEN) {
            _cover->open();
        } else if (state == CLOSE) {
            _cover->close();
        }
    };
};

#endif //COVERCONTROLLER_H
