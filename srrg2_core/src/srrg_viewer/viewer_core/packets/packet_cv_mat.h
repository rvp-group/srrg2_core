#pragma once
#include "opencv2/core.hpp"
#include "packet_base.h"

namespace srrg2_core {

  //! @brief evil opencv matrix packager
  template <uint8_t PacketType_>
  struct PacketCvMat_ : public PacketBase {
    template <typename T_>
    friend class PacketCreator_;
    static constexpr uint8_t PacketType = PacketType_;
    using PayloadType                   = cv::Mat;
    PayloadType data;

    // ds data header (mirrored from opencv)
    int matrix_type = -1; // ds type is already taken by base NGHGNNGNGNGN
    int channels = -1;
    int rows     = -1;
    int cols     = -1;

    PacketCvMat_(const PayloadType& data_ = PayloadType()) :
      PacketBase(PacketType_),
      data(data_),
      matrix_type(data.type()),
      channels(data.channels()),
      rows(data.rows),
      cols(data.cols) {
    }
    virtual ~PacketCvMat_() {
    }
    char* serialize(char* buffer) const override {
      assert(buffer);

      // ds copy matrix header information
      std::memcpy((void*) buffer, (const void*) &matrix_type, sizeof(int));
      assert(matrix_type != -1);
      buffer += sizeof(int);
      std::memcpy((void*) buffer, (const void*) &channels, sizeof(int));
      assert(channels > 0);
      buffer += sizeof(int);
      std::memcpy((void*) buffer, (const void*) &rows, sizeof(int));
      assert(rows > 0);
      buffer += sizeof(int);
      std::memcpy((void*) buffer, (const void*) &cols, sizeof(int));
      assert(cols > 0);
      buffer += sizeof(int);

      // ds determine data write size
      const size_t number_of_data_bytes_to_write(getNumberOfDataBytes(matrix_type));
      std::memcpy((void*) buffer, (const void*) data.data, number_of_data_bytes_to_write);
      buffer += number_of_data_bytes_to_write;
      return buffer;
    }
    const char* deserialize(const char* buffer) override {
      assert(buffer);
      matrix_type = -1;
      channels    = -1;
      rows        = -1;
      cols        = -1;

      // ds retrieve header information
      std::memcpy((void*) &matrix_type, (const void*) buffer, sizeof(int));
      assert(matrix_type != -1);
      buffer += sizeof(int);
      std::memcpy((void*) &channels, (const void*) buffer, sizeof(int));
      assert(channels > 0);
      buffer += sizeof(int);
      std::memcpy((void*) &rows, (const void*) buffer, sizeof(int));
      assert(rows > 0);
      buffer += sizeof(int);
      std::memcpy((void*) &cols, (const void*) buffer, sizeof(int));
      assert(cols > 0);
      buffer += sizeof(int);

      // ds initialize matrix
      data = cv::Mat(rows, cols, matrix_type);
      assert(data.channels() == channels);

      // ds determine data read size
      const size_t number_of_data_bytes_to_read(getNumberOfDataBytes(matrix_type));
      std::memcpy((void*) data.data, (const void*) buffer, number_of_data_bytes_to_read);
      buffer += number_of_data_bytes_to_read;
      return buffer;
    }

    size_t getNumberOfDataBytes(const int& matrix_type_) const {
      size_t number_of_data_bytes = 0;

      // ds conversion from: http://dovgalecs.com/blog/opencv-matrix-types/
      switch (matrix_type_) {
        case CV_8UC1:
        case CV_8UC2:
        case CV_8UC3:
        case CV_8UC4: {
          number_of_data_bytes = channels * rows * cols * sizeof(uchar);
          break;
        }
        case CV_8SC1:
        case CV_8SC2:
        case CV_8SC3:
        case CV_8SC4: {
          number_of_data_bytes = channels * rows * cols * sizeof(schar);
          break;
        }
        case CV_16UC1:
        case CV_16UC2:
        case CV_16UC3:
        case CV_16UC4: {
          number_of_data_bytes = channels * rows * cols * sizeof(ushort);
          break;
        }
        case CV_16SC1:
        case CV_16SC2:
        case CV_16SC3:
        case CV_16SC4: {
          number_of_data_bytes = channels * rows * cols * sizeof(short);
          break;
        }
        case CV_32SC1:
        case CV_32SC2:
        case CV_32SC3:
        case CV_32SC4: {
          number_of_data_bytes = channels * rows * cols * sizeof(int);
          break;
        }
        case CV_32FC1:
        case CV_32FC2:
        case CV_32FC3:
        case CV_32FC4: {
          number_of_data_bytes = channels * rows * cols * sizeof(float);
          break;
        }
        case CV_64FC1:
        case CV_64FC2:
        case CV_64FC3:
        case CV_64FC4: {
          number_of_data_bytes = channels * rows * cols * sizeof(double);
          break;
        }
        default: {
          throw std::runtime_error("PacketCvMat::getNumberOfDataBytes|unsupported cv::Mat type: " +
                                   std::to_string(matrix_type_));
        }
      }
      assert(number_of_data_bytes > 0);
      return number_of_data_bytes;
    }
  };

  //! from packets.h
  //! - 0xC*      -> OpenCV sickness
  static constexpr uint8_t PACKET_TYPE_CV_MAT = 0xC0;

  using PacketCvMat = PacketCvMat_<PACKET_TYPE_CV_MAT>;

} // namespace srrg2_core
