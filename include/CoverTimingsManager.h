#ifndef COVERTIMINGSMANAGER_H
#define COVERTIMINGSMANAGER_H


#include <optional>

struct CoverTimingsEntry {
    unsigned char speed;
    unsigned long timeToMove;
};


class ICoverTimingsManager {
public:
    virtual void saveTimeToOpen(CoverTimingsEntry) = 0;
    virtual void saveTimeToClose(CoverTimingsEntry) = 0;
    virtual std::optional<unsigned long> loadTimeToOpen() = 0;
    virtual std::optional<unsigned long> loadTimeToClose() = 0;

    virtual ~ICoverTimingsManager() = default;
};

class CoverTimingsManager : public ICoverTimingsManager {
private:
    CoverTimingsEntry toOpen{}, toClose{};
    bool valid = false;

public:
    CoverTimingsManager();
    void saveTimeToOpen(CoverTimingsEntry) override;
    void saveTimeToClose(CoverTimingsEntry) override;
    std::optional<unsigned long> loadTimeToOpen() override;
    std::optional<unsigned long> loadTimeToClose() override;
};

#endif //COVERTIMINGSMANAGER_H
