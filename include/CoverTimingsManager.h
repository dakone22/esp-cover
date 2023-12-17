#ifndef COVERTIMINGSMANAGER_H
#define COVERTIMINGSMANAGER_H


#include <optional>

/**
 * @brief Структура, представляющая запись тайминга штор.
 */
struct CoverTimingsEntry {
    unsigned char speed; /**< Скорость моторов. */
    unsigned long timeToMove; /**< Время, необходимое для перемещения шторы. */
};

/**
 * @brief Интерфейс для управления таймингом штор: сохранение и загрузка значений времени открытия и времени закрытия.
 */
class ICoverTimingsManager {
public:
    /**
     * @brief Сохраняет запись тайминга времени открытия шторы.
     * @param entry Запись тайминга шторы для сохранения.
     */
    virtual void saveTimeToOpen(CoverTimingsEntry entry) = 0;

    /**
     * @brief Сохраняет запись тайминга времени до закрытия шторы.
     * @param entry Запись тайминга шторы для сохранения.
     */
    virtual void saveTimeToClose(CoverTimingsEntry entry) = 0;

    /**
     * @brief Загружает запись тайминга времени до открытия крышки.
     * @return Опциональное значение, содержащее загруженное значение времени до открытия, если оно доступно, пустое в противном случае.
     */
    virtual std::optional<unsigned long> loadTimeToOpen() = 0;

    /**
     * @brief Загружает запись тайминга времени до закрытия крышки.
     * @return Необязательный параметр, содержащий загруженное значение времени до закрытия, если оно доступно, и пустой в противном случае.
     */
    virtual std::optional<unsigned long> loadTimeToClose() = 0;

    virtual ~ICoverTimingsManager() = default;
};

/**
 * @brief Конкретная реализация интерфейса ICoverTimingsManager.
 */
class CoverTimingsManager : public ICoverTimingsManager {
private:
    CoverTimingsEntry toOpen{}; /**< Тайминг для открытия. */
    CoverTimingsEntry toClose{}; /**< Тайминг для закрытия. */
    bool valid = false; /**< Флаг, указывающий, действительны ли тайминги. */

    int getOpenEntryAddress() const;
    int getCloseEntryAddress() const;

    int index;

public:
    CoverTimingsManager();

    void saveTimeToOpen(CoverTimingsEntry) override;
    void saveTimeToClose(CoverTimingsEntry) override;

    std::optional<unsigned long> loadTimeToOpen() override;
    std::optional<unsigned long> loadTimeToClose() override;

    static unsigned int GLOBAL_INDEX;
};

unsigned int CoverTimingsManager::GLOBAL_INDEX = 0;

#endif //COVERTIMINGSMANAGER_H
