#include "viewer_canvas.h"

namespace srrg2_core {

#define DEFINE_METHOD(METHOD_NAME_, PARAM_TYPE_)                   \
  void ViewerCanvas::putPoints(const PARAM_TYPE_& points_) {       \
    if (!_setup())                                                 \
      return;                                                      \
    Packet##PARAM_TYPE_* pack = new Packet##PARAM_TYPE_(&points_); \
    _serializer.putPacket(pack->type, pack);                       \
  }

  ViewerCanvas::~ViewerCanvas() {
  }

  // --------------------------------------------------------------------------
  // --------------------- vector point cloud stuff ---------------------------
  // --------------------------------------------------------------------------
  DEFINE_METHOD(putPoints, PointIntensityDescriptor2fVectorCloud)
  DEFINE_METHOD(putPoints, PointIntensityDescriptor3fVectorCloud)

  DEFINE_METHOD(putPoints, Point2fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormal2fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormalColor2fVectorCloud)

  DEFINE_METHOD(putPoints, Point3fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormal3fVectorCloud)
  // DEFINE_METHOD(putPoints, PointNormalCurvature3fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormalColor3fVectorCloud)

  DEFINE_METHOD(putPoints, Point4fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormal4fVectorCloud)
  DEFINE_METHOD(putPoints, PointNormalColor4fVectorCloud)

  DEFINE_METHOD(putPoints, Point2fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormal2fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormalColor2fMatrixCloud)

  DEFINE_METHOD(putPoints, Point3fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormal3fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormalColor3fMatrixCloud)

  DEFINE_METHOD(putPoints, Point4fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormal4fMatrixCloud)
  DEFINE_METHOD(putPoints, PointNormalColor4fMatrixCloud)

  void ViewerCanvas::putPolygon(const PointNormalColor3fVectorCloud& points_) {
    if (!_setup())
      return;

    // ia pack the payload
    PacketPolygonPointNormalColor3fVectorCloud* pack =
      new PacketPolygonPointNormalColor3fVectorCloud(&points_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putPolygonWireframe(const PointNormalColor3fVectorCloud& points_) {
    if (!_setup())
      return;

    // ia pack the payload
    PacketPolygonWireframePointNormalColor3fVectorCloud* pack =
      new PacketPolygonWireframePointNormalColor3fVectorCloud(&points_);
    _serializer.putPacket(pack->type, pack);
  }

  // --------------------------------------------------------------------------
  // --------------------------- everything else ------------------------------
  // --------------------------------------------------------------------------
  void
  ViewerCanvas::putPoints(const int& size_, const Vector3f* points_, const Vector3f* normals_) {
    if (!_setup())
      return;

    // ia pack the payload
    PacketPayloadPoints* pack = new PacketPayloadPoints(size_, points_, normals_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putLine(const int& size_, const Vector3f* points_, const Vector3f* normals_) {
    if (!_setup())
      return;

    // ia pack the payload
    PacketPayloadLines* pack = new PacketPayloadLines(size_, points_, normals_);
    _serializer.putPacket(pack->type, pack);
  }

  void
  ViewerCanvas::putSegment(const int& size_, const Vector3f* points_, const Vector3f* normals_) {
    if (!_setup())
      return;

    // ia pack the payload
    PacketPayloadSegments* pack = new PacketPayloadSegments(size_, points_, normals_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putSphere(const float& radius_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectSphere* pack = new PacketObjectSphere(radius_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putPlane(const Vector2f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectPlane* pack = new PacketObjectPlane(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putBox(const Vector3f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectBox* pack = new PacketObjectBox(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putBoxWireframe(const Vector3f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectBoxWireframe* pack = new PacketObjectBoxWireframe(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putPyramid(const Vector2f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectPyramid* pack = new PacketObjectPyramid(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putPyramidWireframe(const Vector2f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectPyramidWireframe* pack = new PacketObjectPyramidWireframe(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putEllipsoid(const Vector3f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectEllipsoid* pack = new PacketObjectEllipsoid(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putCone(const Vector2f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectCone* pack = new PacketObjectCone(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putDisk(const float& radius_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectDisk* pack = new PacketObjectDisk(radius_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putCylinder(const Vector2f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectCylinder* pack = new PacketObjectCylinder(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putArrow2D(const Vector3f& size_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectArrow2D* pack = new PacketObjectArrow2D(size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putReferenceSystem(const float& rf_linewidth_) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketObjectReferenceFrame* pack = new PacketObjectReferenceFrame(rf_linewidth_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putText(const std::string& text_) {
    if (!_setup())
      return;
    // ia create a packet object
    PacketObjectText* pack = new PacketObjectText(text_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::multMatrix(const Matrix4f& m) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketTransformMultMatrix* pack = new PacketTransformMultMatrix(m);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::scale(const Vector3f& scales) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketTransformMultScale* pack = new PacketTransformMultScale(scales);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::rotate(const Matrix3f& rotation_matrix) {
    if (!_setup())
      return;

    // ia create a packet object
    PacketTransformMultRotation* pack = new PacketTransformMultRotation(rotation_matrix);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::translate(const Vector3f& translation) {
    if (!_setup())
      return;
    // ia create a packet object
    PacketTransformMultTraslation* pack = new PacketTransformMultTraslation(translation);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::pushMatrix() {
    if (!_setup())
      return;

    // ia pack the command
    PacketCommandPushMatrix* pack = new PacketCommandPushMatrix();
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::popMatrix() {
    if (!_setup())
      return;

    // ia pack the command
    PacketCommandPopMatrix* pack = new PacketCommandPopMatrix();
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::setPointSize(const float& point_size_) {
    if (!_setup())
      return;

    // ia pack the attribute
    PacketAttributePointSize* pack = new PacketAttributePointSize(point_size_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::setLineWidth(const float& line_width_) {
    if (!_setup())
      return;

    // ia pack the attribute
    PacketAttributeLineWidth* pack = new PacketAttributeLineWidth(line_width_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::setColor(const Vector3f& color_) {
    if (!_setup())
      return;

    // ia pack the attribute
    PacketAttributeColorRGB* pack = new PacketAttributeColorRGB(color_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::setColor(const Vector4f& color_) {
    if (!_setup())
      return;

    // ia pack the attribute
    PacketAttributeColorRGBA* pack = new PacketAttributeColorRGBA(color_);
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::pushPointSize() {
    if (!_setup())
      return;

    // ia pack the command
    PacketCommandPushPointSize* pack = new PacketCommandPushPointSize();
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::pushLineWidth() {
    if (!_setup())
      return;

    // ia pack the command
    PacketCommandPushLineWidth* pack = new PacketCommandPushLineWidth();
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::pushColor() {
    if (!_setup())
      return;

    // ia pack the command
    PacketCommandPushColor* pack = new PacketCommandPushColor();
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::popAttribute() {
    if (!_setup())
      return;

    // ia pack the command
    PacketCommandPopAttribute* pack = new PacketCommandPopAttribute();
    _serializer.putPacket(pack->type, pack);
  }

  void ViewerCanvas::putVisualMatchables(const VisualMatchablefVector& matchables_) {
    if (!_setup())
      return;

    // ia pack the command
    PacketVisualMatchablefVector* pack = new PacketVisualMatchablefVector(&matchables_);
    _serializer.putPacket(pack->type, pack);
  }

  // ds FIXME
  void ViewerCanvas::putImage(const cv::Mat& image_) {
    if (!_setup()) {
      return;
    }
    PacketCvMat* package = new PacketCvMat(image_);
    _serializer.putPacket(package->type, package);
  }

  // ia TODO workaround
  void ViewerCanvas::putPoints(const PointIntensity3fVectorCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    Point3fVectorCloud copy;
    copy.reserve(points_.size());

    for (const auto& p : points_) {
      Point3f copy_p;
      copy_p.coordinates() = p.coordinates();
      copy.emplace_back(copy_p);
    }

    // ia packet
    PacketPoint3fVectorCloud* packet = new PacketPoint3fVectorCloud(&copy);
    _serializer.putPacket(packet->type, packet);
  }

  // ia TODO workaround
  void ViewerCanvas::putPoints(const PointNormalCurvatureColor3fVectorCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    PointNormalColor3fVectorCloud copy;
    copy.reserve(points_.size());

    for (const auto& p : points_) {
      PointNormalColor3f copy_p;
      copy_p.coordinates() = p.coordinates();
      copy_p.normal()      = p.normal();
      copy_p.color()       = p.color();
      copy.emplace_back(copy_p);
    }

    // ia packet
    PacketPointNormalColor3fVectorCloud* packet = new PacketPointNormalColor3fVectorCloud(&copy);
    _serializer.putPacket(packet->type, packet);
  }
  void ViewerCanvas::putPoints(const PointNormalCurvature3fVectorCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    PointNormalColor3fVectorCloud copy;
    copy.reserve(points_.size());
    for (const auto& p : points_) {
      PointNormalColor3f copy_p;
      Vector3f color       = Vector3f::Zero();
      copy_p.coordinates() = p.coordinates();
      copy_p.normal()      = p.normal();
      if (p.curvature() < 0.25) {
        color = ColorPalette::color3fWhite();
      } else if (p.curvature() < 0.75) {
        color = ColorPalette::color3fLightGray();
      } else {
        color = ColorPalette::color3fBlack();
      }
      copy_p.color() = color;
      copy.emplace_back(copy_p);
    }

    // ia packet
    PacketPointNormalColor3fVectorCloud* packet = new PacketPointNormalColor3fVectorCloud(&copy);
    _serializer.putPacket(packet->type, packet);
  }

  // ia TODO workaround
  void ViewerCanvas::putPoints(const PointIntensity3fMatrixCloud& points_) {
    if (!_setup()) {
      return;
    }

    // ia copy everything in a normal vector cloud
    Point3fVectorCloud copy;
    copy.reserve(points_.size());

    for (auto it = points_.begin(); it != points_.end(); ++it) {
      if (it->status == POINT_STATUS::Invalid) {
        continue;
      }
      Point3f copy_p;
      copy_p.coordinates() = it->coordinates();
      copy.emplace_back(copy_p);
    }

    // ia packet
    PacketPoint3fVectorCloud* packet = new PacketPoint3fVectorCloud(&copy);
    _serializer.putPacket(packet->type, packet);
  }

  // ia TODO workaround
  void ViewerCanvas::putPoints(const PointNormalCurvatureColor3fMatrixCloud& points_) {
    if (!_setup()) {
      return;
    }
    // ia copy everything in a normal vector cloud
    PointNormalColor3fVectorCloud copy;
    copy.reserve(points_.size());

    for (auto it = points_.begin(); it != points_.end(); ++it) {
      PointNormalColor3f copy_p;
      copy_p.coordinates() = it->coordinates();
      copy_p.normal()      = it->normal();
      copy_p.color()       = it->color();
      copy.emplace_back(copy_p);
    }

    // ia packet
    PacketPointNormalColor3fVectorCloud* packet = new PacketPointNormalColor3fVectorCloud(&copy);
    _serializer.putPacket(packet->type, packet);
  }

  void ViewerCanvas::flush() {
    if (!_setup())
      return;

    // ia send an end epoch
    PacketInfoEndEpoch* pack = new PacketInfoEndEpoch();
    _serializer.putPacket(pack->type, pack);

    // ia throw the buffer somewhere
    param_buffer_sink_ptr->putBuffer(_current_buffer);
    _current_buffer = 0;
  }

  bool ViewerCanvas::_setup() {
    // ia if nobody is attached to this canvas, simply do nothing (bench mode)
    if (!param_buffer_sink_ptr.value())
      return false;

    // ia check connection to the viewport (if its up again maybe)
    param_buffer_sink_ptr.value()->checkConnection(); // ia this check is heavy :(

    if (!param_buffer_sink_ptr->isActive())
      return false;

    if (!param_buffer_manager_ptr.value())
      throw std::runtime_error("ViewerCanvas::_setup|please set a valid BufferManager");

    // ia check if I have a buffer otherwise get one
    if (!_current_buffer) {
      _current_buffer = param_buffer_manager_ptr->getBuffer();
      _serializer.setBuffer(_current_buffer);
    }

    return true;
  }

} // namespace srrg2_core
