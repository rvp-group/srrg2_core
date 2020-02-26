#include "instances.h"

namespace srrg2_core {

  void messages_registerTypes() {
    BOSS_REGISTER_CLASS(PointCloud2DataBLOBReference); //bdc, no idea if it should be CLASS or BLOB

    BOSS_REGISTER_CLASS(CameraInfoMessage);
    BOSS_REGISTER_CLASS(ImageMessage);
    BOSS_REGISTER_CLASS(IMUMessage);
    BOSS_REGISTER_CLASS(LaserMessage);
    BOSS_REGISTER_CLASS(OdometryMessage);
    BOSS_REGISTER_CLASS(PointStampedMessage);
    BOSS_REGISTER_CLASS(RangeMessage);
    BOSS_REGISTER_CLASS(TicksMessage);
    BOSS_REGISTER_CLASS(TransformEventsMessage);
    BOSS_REGISTER_CLASS(TwistStampedMessage);
    BOSS_REGISTER_CLASS(PointCloud2Message);
    BOSS_REGISTER_CLASS(JointsMessage);

    BOSS_REGISTER_CLASS(MessagePack);
    BOSS_REGISTER_CLASS(MessageFileSink);
    BOSS_REGISTER_CLASS(MessageFileSource);
    BOSS_REGISTER_CLASS(MessageSortedSource);
    BOSS_REGISTER_CLASS(MessageSynchronizedSource);
    BOSS_REGISTER_CLASS(MessageSourcePlatform);
    BOSS_REGISTER_CLASS(MessageOdomSubsamplerSource);
  }
}
