#ifndef GRINDINGAPP_SRC_UI_COMMANDS_H_
#define GRINDINGAPP_SRC_UI_COMMANDS_H_

#include <functional>

#include <QUndoCommand>
#include <gp_Trsf.hxx>

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

class CmdMoveBase : public QUndoCommand
{
public:
    CmdMoveBase(RobotController* controller,
                const gp_Trsf& old_trsf, const gp_Trsf& new_trsf,
                QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;
    int id() const override { return 2; }

private:
    RobotController* controller_;
    gp_Trsf old_trsf_;
    gp_Trsf new_trsf_;
};

class CmdMoveWorkpiece : public QUndoCommand
{
public:
    using ApplyFn = std::function<void(const gp_Trsf&)>;

    CmdMoveWorkpiece(ApplyFn fn,
                     const gp_Trsf& old_trsf, const gp_Trsf& new_trsf,
                     QUndoCommand* parent = nullptr);

    void undo() override;
    void redo() override;
    int id() const override { return 3; }

private:
    ApplyFn apply_fn_;
    gp_Trsf old_trsf_;
    gp_Trsf new_trsf_;
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_COMMANDS_H_
