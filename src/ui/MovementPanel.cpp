#include "MovementPanel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>

#include "RobotDisplay.h"

namespace nl {
namespace ui {

MovementPanel::MovementPanel(const QString& target_name,
	const gp_Trsf& initial_trsf,
	QWidget* parent)
	: QDockWidget(parent)
	, initial_trsf_(initial_trsf)
{
	SetupUi(target_name);

	// Fill spinboxes from initial transform
	nl::utils::Vector3d rpy, pos;
	nl::occ::TrsfToRpyPos(initial_trsf_, rpy, pos);

	spinboxes_[0]->setValue(pos.x);
	spinboxes_[1]->setValue(pos.y);
	spinboxes_[2]->setValue(pos.z);
	spinboxes_[3]->setValue(rpy.x);
	spinboxes_[4]->setValue(rpy.y);
	spinboxes_[5]->setValue(rpy.z);
}

void MovementPanel::SetupUi(const QString& target_name)
{
	setWindowTitle(tr("Move: %1").arg(target_name));
	setFeatures(QDockWidget::DockWidgetClosable);

	auto* group = new QGroupBox;
	auto* grid = new QGridLayout;

	const char* labels[] = { "X", "Y", "Z", "RX", "RY", "RZ" };
	const char* units[] = { "mm", "mm", "mm", "\xC2\xB0", "\xC2\xB0", "\xC2\xB0" };

	for (int i = 0; i < 6; ++i) {
		grid->addWidget(new QLabel(labels[i]), i, 0);

		spinboxes_[i] = new QDoubleSpinBox;
		spinboxes_[i]->setDecimals(3);
		if (i < 3) {
			spinboxes_[i]->setRange(-10000.0, 10000.0);
		}
		else {
			spinboxes_[i]->setRange(-180.0, 180.0);
		}
		spinboxes_[i]->setSingleStep(1.0);
		grid->addWidget(spinboxes_[i], i, 1);

		grid->addWidget(new QLabel(units[i]), i, 2);

		connect(spinboxes_[i], &QDoubleSpinBox::editingFinished,
			this, &MovementPanel::OnSpinBoxEditingFinished);
	}

	group->setLayout(grid);

	ok_btn_ = new QPushButton(tr("OK"));
	cancel_btn_ = new QPushButton(tr("Cancel"));
	
	auto* btn_layout = new QHBoxLayout;
	btn_layout->addStretch();
	btn_layout->addWidget(ok_btn_);
	btn_layout->addWidget(cancel_btn_);

	auto* main_layout = new QVBoxLayout;
	main_layout->addWidget(group);
	main_layout->addLayout(btn_layout);
	main_layout->addStretch();

	auto* container = new QWidget;
	container->setLayout(main_layout);
	setWidget(container);

	connect(ok_btn_, &QPushButton::clicked, this, [this]() {
		emit Accepted(ComputeTrsf());
		});
	connect(cancel_btn_, &QPushButton::clicked, this, [this]() {
		emit Cancelled();
		});
}

void MovementPanel::OnSpinBoxEditingFinished()
{
	emit PreviewRequested(ComputeTrsf());
}

gp_Trsf MovementPanel::ComputeTrsf() const
{
	nl::utils::Vector3d pos{
		spinboxes_[0]->value(),
		spinboxes_[1]->value(),
		spinboxes_[2]->value()
	};
	nl::utils::Vector3d rpy{
		spinboxes_[3]->value(),
		spinboxes_[4]->value(),
		spinboxes_[5]->value()
	};
	return nl::occ::RpyPosTrsf(rpy, pos);
}

} // namespace ui
} // namespace nl