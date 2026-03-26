#ifndef GRINDINGAPP_SRC_UI_TRAJECTORY_PLAYER_H_
#define GRINDINGAPP_SRC_UI_TRAJECTORY_PLAYER_H_

#include <QDockWidget>

#include "GrindingUIExport.h"

class QTimer;
class QPushButton;
class QSlider;
class QLabel;

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT TrajectoryPlayer : public QDockWidget {
    Q_OBJECT

public:
    explicit TrajectoryPlayer(QWidget* parent = nullptr);

    void SetFrameCount(int count);
    void SetCurrentFrame(int frame);

signals:
    void FrameChanged(int index);
    void PlaybackFinished();

private slots:
    void OnPlay();
    void OnPause();
    void OnStop();
    void OnTimerTick();
    void OnSliderMoved(int value);
    void OnSpeedChanged(int value);

private:
    void SetupUi();
    void UpdateLabel();

    QTimer*      timer_ = nullptr;
    QPushButton* play_btn_ = nullptr;
    QPushButton* pause_btn_ = nullptr;
    QPushButton* stop_btn_ = nullptr;
    QSlider*     progress_slider_ = nullptr;
    QSlider*     speed_slider_ = nullptr;
    QLabel*      frame_label_ = nullptr;
    QLabel*      speed_label_ = nullptr;

    int current_frame_ = 0;
    int frame_count_ = 0;
    double speed_factor_ = 1.0;
    static constexpr int kBaseIntervalMs = 33;
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_TRAJECTORY_PLAYER_H_
