#include "Commands.h"

namespace nl {
namespace ui {

const int CMD_ID_JOG_JOINTS = 1;

CmdJogJoints::CmdJogJoints(RobotController* controller, const nl::utils::Q& old_angles, const nl::utils::Q& new_angles, QUndoCommand* parent)
    : QUndoCommand(parent), controller_(controller), old_angles_(old_angles), new_angles_(new_angles)
{
    setText("Jog Joints");
}

void CmdJogJoints::undo()
{
    // Temporarily block signals or let it flow so UI updates, but avoid creating another command
    controller_->SetJointAngles(old_angles_);
}

void CmdJogJoints::redo()
{
    controller_->SetJointAngles(new_angles_);
}

int CmdJogJoints::id() const
{
    return CMD_ID_JOG_JOINTS;
}

bool CmdJogJoints::mergeWith(const QUndoCommand* other)
{
    if (other->id() != id())
        return false;

    const CmdJogJoints* other_cmd = static_cast<const CmdJogJoints*>(other);
    // Keep the original old_angles_, but update to the latest new_angles_
    new_angles_ = other_cmd->new_angles_;
    return true;
}

// --- CmdMoveBase ---

const int CMD_ID_MOVE_BASE = 2;

CmdMoveBase::CmdMoveBase(RobotController* controller,
                         const gp_Trsf& old_trsf, const gp_Trsf& new_trsf,
                         QUndoCommand* parent)
    : QUndoCommand(parent), controller_(controller), old_trsf_(old_trsf), new_trsf_(new_trsf)
{
    setText("Move Robot Base");
}

void CmdMoveBase::undo()
{
    controller_->SetBaseTrsf(old_trsf_);
}

void CmdMoveBase::redo()
{
    controller_->SetBaseTrsf(new_trsf_);
}

// --- CmdMoveWorkpiece ---

const int CMD_ID_MOVE_WORKPIECE = 3;

CmdMoveWorkpiece::CmdMoveWorkpiece(ApplyFn fn,
                                   const gp_Trsf& old_trsf, const gp_Trsf& new_trsf,
                                   QUndoCommand* parent)
    : QUndoCommand(parent), apply_fn_(std::move(fn)), old_trsf_(old_trsf), new_trsf_(new_trsf)
{
    setText("Move Workpiece");
}

void CmdMoveWorkpiece::undo()
{
    apply_fn_(old_trsf_);
}

void CmdMoveWorkpiece::redo()
{
    apply_fn_(new_trsf_);
}

} // namespace ui
} // namespace nl
