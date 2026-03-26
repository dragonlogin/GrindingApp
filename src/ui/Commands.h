#ifndef GRINDINGAPP_SRC_UI_COMMANDS_H_
#define GRINDINGAPP_SRC_UI_COMMANDS_H_

#include <QUndoCommand>
#include "Q.h"
#include "RobotController.h"

namespace nl {
namespace ui {

class CmdJogJoints : public QUndoCommand
{
public:
    CmdJogJoints(RobotController* controller, const nl::utils::Q& old_angles, const nl::utils::Q& new_angles, QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;
    int id() const override;
    bool mergeWith(const QUndoCommand* other) override;

private:
    RobotController* controller_;
    nl::utils::Q old_angles_;
    nl::utils::Q new_angles_;
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_COMMANDS_H_
