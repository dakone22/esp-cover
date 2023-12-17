#ifndef COVER_H
#define COVER_H

#include <memory>

/**
 * @brief Enum, представляющий состояния шторы.
 */
enum class CoverState {
    Opened, Closed, Opening, Closing, Unknown,
};

/**
 * @brief Интерфейс для операций, связанных с состоянием шторы.
 */
class IStateCover{
public:
    virtual void open() = 0;
    virtual void close() = 0;
    virtual void stop() = 0;

    virtual CoverState getState() = 0;

    virtual ~IStateCover() = default;
};

/**
 * @brief Интерфейс для позиционных операций шторы.
 */
class IPositionCover : public IStateCover{
public:
    /**
     * @brief Константа, представляющая неизвестную позицию.
     */
    static const int UnknownPosition = -1;

    /**
     * @brief Установить целевое положение шторы.
     * @param value Целевое положение для установки.
     */
    virtual void setTargetPosition(int value) = 0;

    /**
     * @brief Целевая позиция шторы.
     * @return Целевая позиция.
     */
    virtual int getTargetPosition() = 0;

    /**
     * @brief Установить текущее положение шторы.
     * @param value Текущая позиция для установки.
     */
    virtual void setCurrentPosition(int value) = 0;

    /**
     * @brief Текущая позиция шторы.
     * @return Текущая позиция.
     */
    virtual int getCurrentPosition() = 0;

    /**
     * @brief Получает положение, в котором крышка полностью открыта.
     * @return Открытая позиция.
     */
    virtual int getOpenedPosition() = 0;

    /**
     * @brief Получает положение, в котором крышка полностью закрыта.
     * @return Закрытая позиция.
     */
    virtual int getClosedPosition() = 0;

    /**
     * @brief Открывает штору, устанавливая положение цели в положение открытия.
     */
    inline void open() override { setTargetPosition(getOpenedPosition()); };

    /**
     * @brief Закрывает штору, устанавливая целевое положение в закрытое положение.
     */
    inline void close() override { setTargetPosition(getClosedPosition()); };

    /**
     * @brief Остановка шторы путём установки целевой позиции на текущую позицию.
     */
    inline void stop() override {
        setTargetPosition(getCurrentPosition());
        setCurrentPosition(getCurrentPosition());
    };

    /**
     * @brief Получить состояние шторы.
     * @return CoverState, представляющий текущее состояние.
     */
    CoverState getState() override;

    /**
     * @brief Проверяет, указывает ли целевая позиция в сторону открытого состояния.
     * @return True, если крышка нацелена в сторону открытого состояния, в противном случае — false.
     */
    inline bool isTargetToOpen() {
        if (getOpenedPosition() > getClosedPosition()) {
            return getTargetPosition() > getCurrentPosition();
        } else {
            return getTargetPosition() < getCurrentPosition();
        }
    }

    /**
     * @brief Проверяет, указывает ли целевая позиция в сторону закрытого состояния.
     * @return True, если обложка нацелена в сторону закрытого состояния, в противном случае — false.
     */
    inline bool isTargetToClose() {
        if (getOpenedPosition() > getClosedPosition()) {
            return getTargetPosition() < getCurrentPosition();
        } else {
            return getTargetPosition() > getCurrentPosition();
        }
    }
};

/**
 * @brief Конкретная реализация интерфейса IPositionCover.
 */
class Cover : public IPositionCover {
private:
    int currentPosition = UnknownPosition;
    int targetPosition = UnknownPosition;
    int closedPosition;
    int openedPosition;

public:
    /**
     * @brief Конструктор класса Cover.
     * @param closedPosition Положение, при котором штора полностью закрыта.
     * @param openedPosition Положение, при котором штора полностью открыта.
     */
    Cover(int closedPosition, int openedPosition);

    inline void setTargetPosition(int value) override { targetPosition = value; }
    inline int getTargetPosition() override { return targetPosition; }

    inline void setCurrentPosition(int value) override { currentPosition = value; }
    inline int getCurrentPosition() override { return currentPosition; }

    inline int getOpenedPosition() override { return openedPosition; }
    inline int getClosedPosition() override { return closedPosition; }
};

#endif