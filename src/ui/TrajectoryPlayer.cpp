#include "TrajectoryPlayer.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QSlider>
#include <QLabel>

namespace nl {
namespace ui {

TrajectoryPlayer::TrajectoryPlayer(QWidget* parent)
    : QDockWidget(tr("Playback"), parent)
{
    SetupUi();
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &TrajectoryPlayer::OnTimerTick);
}

void TrajectoryPlayer::SetupUi()
{
    auto* widget = new QWidget;
    auto* layout = new QHBoxLayout(widget);
    layout->setContentsMargins(6, 2, 6, 2);

    // Transport buttons
    play_btn_ = new QPushButton(tr("\u25B6"));
    pause_btn_ = new QPushButton(tr("\u23F8"));
    stop_btn_ = new QPushButton(tr("\u23F9"));

    play_btn_->setFixedWidth(36);
    pause_btn_->setFixedWidth(36);
    stop_btn_->setFixedWidth(36);

    connect(play_btn_, &QPushButton::clicked, this, &TrajectoryPlayer::OnPlay);
    connect(pause_btn_, &QPushButton::clicked, this, &TrajectoryPlayer::OnPause);
    connect(stop_btn_, &QPushButton::clicked, this, &TrajectoryPlayer::OnStop);

    layout->addWidget(play_btn_);
    layout->addWidget(pause_btn_);
    layout->addWidget(stop_btn_);

    // Progress slider
    progress_slider_ = new QSlider(Qt::Horizontal);
    progress_slider_->setRange(0, 0);
    connect(progress_slider_, &QSlider::sliderMoved,
            this, &TrajectoryPlayer::OnSliderMoved);
    layout->addWidget(progress_slider_, 1);

    // Frame label
    frame_label_ = new QLabel("0 / 0");
    frame_label_->setMinimumWidth(80);
    layout->addWidget(frame_label_);

    // Speed slider
    layout->addWidget(new QLabel(tr("Speed:")));
    speed_slider_ = new QSlider(Qt::Horizontal);
    speed_slider_->setRange(1, 30);  // 0.1x to 3.0x
    speed_slider_->setValue(10);     // 1.0x default
    speed_slider_->setFixedWidth(80);
    connect(speed_slider_, &QSlider::valueChanged,
            this, &TrajectoryPlayer::OnSpeedChanged);
    layout->addWidget(speed_slider_);

    speed_label_ = new QLabel("1.0x");
    speed_label_->setMinimumWidth(36);
    layout->addWidget(speed_label_);

    setWidget(widget);
    setFixedHeight(50);
}

void TrajectoryPlayer::SetFrameCount(int count)
{
    frame_count_ = count;
    current_frame_ = 0;
    progress_slider_->setRange(0, std::max(0, count - 1));
    progress_slider_->setValue(0);
    UpdateLabel();
}

void TrajectoryPlayer::SetCurrentFrame(int frame)
{
    current_frame_ = frame;
    progress_slider_->setValue(frame);
    UpdateLabel();
}

void TrajectoryPlayer::OnPlay()
{
    if (frame_count_ <= 0) return;
    if (current_frame_ >= frame_count_ - 1) {
        current_frame_ = 0;  // Restart from beginning
    }
    int interval = static_cast<int>(kBaseIntervalMs / speed_factor_);
    timer_->start(std::max(1, interval));
}

void TrajectoryPlayer::OnPause()
{
    timer_->stop();
}

void TrajectoryPlayer::OnStop()
{
    timer_->stop();
    current_frame_ = 0;
    progress_slider_->setValue(0);
    UpdateLabel();
    emit FrameChanged(0);
}

void TrajectoryPlayer::OnTimerTick()
{
    if (current_frame_ >= frame_count_ - 1) {
        timer_->stop();
        emit PlaybackFinished();
        return;
    }

    ++current_frame_;
    progress_slider_->setValue(current_frame_);
    UpdateLabel();
    emit FrameChanged(current_frame_);
}

void TrajectoryPlayer::OnSliderMoved(int value)
{
    current_frame_ = value;
    UpdateLabel();
    emit FrameChanged(value);
}

void TrajectoryPlayer::OnSpeedChanged(int value)
{
    speed_factor_ = value / 10.0;
    speed_label_->setText(QString("%1x").arg(speed_factor_, 0, 'f', 1));

    // Update timer interval if playing
    if (timer_->isActive()) {
        int interval = static_cast<int>(kBaseIntervalMs / speed_factor_);
        timer_->setInterval(std::max(1, interval));
    }
}

void TrajectoryPlayer::UpdateLabel()
{
    frame_label_->setText(
        QString("%1 / %2").arg(current_frame_).arg(frame_count_));
}

} // namespace ui
} // namespace nl
