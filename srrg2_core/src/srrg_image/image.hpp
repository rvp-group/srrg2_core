namespace srrg2_core {

  template <class Scalar_, ImageType ImageType_, int NumChannels_>  
  void Image_<Scalar_, ImageType_, NumChannels_>::toCv(cv::Mat& dest_) const {

    dest_.create(this->rows(), this->cols(), ImageType_);
    for (size_t r = 0; r < this->rows(); ++r) {
      CVVecType* dest_ptr = dest_.ptr<CVVecType>(r);
      const EigenVecType* src_ptr = this->rowPtr(r);
      for (size_t c = 0; c < this->cols(); ++c, ++dest_ptr, ++src_ptr) {
        CVVecType& dest_item = *dest_ptr;
        const EigenVecType& src_item = *src_ptr;
        for (int n = 0; n < NumChannels_; ++n) {
          dest_item[n] = src_item[n];
        }
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>  
  void Image_<Scalar_, ImageType_, NumChannels_>::fromCv(const cv::Mat& src_) {
    assert(src_.type() == ImageType_);
    MatrixType::resize(src_.rows, src_.cols);
    for (int r = 0; r < src_.rows; ++r) {
      const CVVecType* src_ptr = src_.ptr<CVVecType>(r);
      EigenVecType* dest_ptr = this->rowPtr(r);
      for (int c = 0; c < src_.cols; ++c, ++dest_ptr, ++src_ptr) {
        const CVVecType& src_item = *src_ptr;
        EigenVecType& dest_item = *dest_ptr;
        for (int n = 0; n < NumChannels_; ++n) {
          dest_item[n] = src_item[n];
        }
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>  
  void Image_<Scalar_, ImageType_, NumChannels_>::fromBuffer(const uint8_t* buffer) {
    const Scalar* src_ptr = reinterpret_cast<const Scalar*>(buffer);
    for (size_t r = 0; r < this->rows(); ++r) {
      EigenVecType* dest_ptr = this->rowPtr(r);
      for (size_t c = 0; c < this->cols(); ++c, ++dest_ptr) {
        EigenVecType& dest_item = *dest_ptr;
        for (int n = 0; n < NumChannels_; ++n, ++src_ptr) {
          dest_item[n] = *src_ptr;
        }
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  template <typename DestImageType_, typename ScaleType_>
  void Image_<Scalar_, ImageType_, NumChannels_>::convertTo(DestImageType_& dest,
                                                            const ScaleType_& s) const {
    assert(dest.numChannels() == numChannels());
    typedef typename DestImageType_::EigenVecType DestVecType;

    dest.resize(rows(), cols());
    for (size_t r = 0; r < dest.rows(); ++r) {
      const EigenVecType* src_ptr = this->rowPtr(r);
      DestVecType* dest_ptr = dest.rowPtr(r);
      for (size_t c = 0; c < cols(); ++c, ++src_ptr, ++dest_ptr) {
        const EigenVecType& src_item = *src_ptr;
        DestVecType& dest_item = *dest_ptr;
        for (int n = 0; n < NumChannels_; ++n) {
          dest_item[n] = src_item[n] * s;
        }
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::scale(float scale_factor_) {
    for (size_t r = 0; r < this->rows(); ++r) {
      for (size_t c = 0; c < this->cols(); ++c) {
        (*this)(r, c) *= scale_factor_;
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::downsample(Image_<Scalar,ImageType_,NumChannels_>& dest,
                                                             DOWNSAMPLE_PIXELS npixels) {
    assert(this->type() == ImageType_);
    
    const float base = sqrt(static_cast<float>(npixels)); //TODO use map between DOWNSAMPLE_PIXELS and its base?
    const float inv_base = 1 / base;
    const float inv_npixels = 1 / (float)npixels;

    int new_rows = this->rows() * inv_base;
    int new_cols = this->cols() * inv_base;

    if (this->rows() % new_rows) {
      throw std::runtime_error("Image::downsample: the rows of dest should divide perfectly the rows of this");
    }
    if (this->cols() % new_cols) {
      throw std::runtime_error("Image::downsample: the cols of dest should divide perfectly the cols of this");
    }

    dest.resize(new_rows, new_cols);
    Eigen::Matrix<double, NumChannels_, 1> tmp;
    int r_idx_start, r_idx_end, c_idx_start, c_idx_end;
    for (int r = 0; r < new_rows; ++r) {
      r_idx_start = r * base;
      r_idx_end = r_idx_start + base;
      for (int c = 0; c < new_cols; ++c) {
        c_idx_start = c * base;
        c_idx_end = c_idx_start + base;
        tmp.setZero();
        for (int rr = r_idx_start; rr < r_idx_end; ++rr) {
          for (int cc = c_idx_start; cc < c_idx_end; ++cc) {
            tmp += (*this)(rr, cc).template cast<double>();
          }
        }
        dest(r, c) = (tmp * inv_npixels).template cast<Scalar>();
      }
    }
  }


  template <class Scalar_, ImageType ImageType_, int NumChannels_>
  void Image_<Scalar_, ImageType_, NumChannels_>::integralImage(Image_<Scalar,ImageType_,NumChannels_>& dest_) {
    const int rows = this->rows();
    const int cols = this->cols();
    dest_.resize(rows, cols);
    Eigen::Matrix<Scalar, NumChannels_, 1> A, F, E;
    A.setZero();
    F.setZero();
    E.setZero();
    for(int r = 0; r < rows; ++r) {
      const Eigen::Matrix<Scalar, NumChannels_, 1>* D = this->rowPtr(r);
      Eigen::Matrix<Scalar, NumChannels_, 1>* I = dest_.rowPtr(r);
      for(int c = 0; c < cols; ++c, ++I, ++D) {

        if(r > 1) {
          E = dest_.at(r-1,c);
        } else {
          E.setZero();
        }

        if(r > 1 && c > 1)
          A = dest_.at(r-1, c-1);
        else
          A.setZero();

        if(c > 1)
          F = dest_.at(r, c-1);
        else
          F.setZero();

        *I = *D + E + F - A;
      }
    }
  }

  template <class Scalar_, ImageType ImageType_, int NumChannels_>  
  void Image_<Scalar_, ImageType_, NumChannels_>::blur(Image_<Scalar,ImageType_,NumChannels_>& dest_,
                                                                const size_t& window_) {
    const int rows = this->rows();
    const int cols = this->cols();
    dest_.resize(rows, cols);
    dest_.fill(Eigen::Matrix<Scalar, NumChannels_, 1>::Zero());
    Image_<Scalar_, ImageType_, NumChannels_> integral;

    this->integralImage(integral);
    for (unsigned int r = window_; r < rows - window_; ++r ) {
      const Eigen::Matrix<Scalar, NumChannels_, 1>* up_row_ptr=integral.rowPtr(r-window_)+window_;
      const Eigen::Matrix<Scalar, NumChannels_, 1>* down_row_ptr=integral.rowPtr(r+window_)+window_;
      Eigen::Matrix<Scalar, NumChannels_, 1>* dest_row_ptr=dest_.rowPtr(r)+window_;

      for (unsigned int c = window_; c < cols - window_;
          ++c, ++down_row_ptr, ++up_row_ptr, ++dest_row_ptr) {
        Eigen::Matrix<Scalar, NumChannels_, 1> m11=*(down_row_ptr+window_);
        Eigen::Matrix<Scalar, NumChannels_, 1> m00=*(up_row_ptr-window_);
        Eigen::Matrix<Scalar, NumChannels_, 1> m01=*(down_row_ptr-window_);
        Eigen::Matrix<Scalar, NumChannels_, 1> m10=*(up_row_ptr+window_);
        Eigen::Matrix<Scalar, NumChannels_, 1> n_sum=m11+m00-m01-m10;
        if (n_sum.dot(n_sum)>0.2)
          *dest_row_ptr = n_sum.normalized();
        else
          *dest_row_ptr = Eigen::Matrix<Scalar, NumChannels_, 1>::Zero();
      }
    }
  }


  template <class Scalar_, ImageType ImageType_>  
  void Image_<Scalar_, ImageType_, 1>::toCv(cv::Mat& dest_) const {
    dest_.create(this->rows(), this->cols(), ImageType_);
    for (size_t r = 0; r < this->rows(); ++r) {
      Scalar* dest_ptr = dest_.ptr<Scalar>(r);
      const Scalar* src_ptr = this->rowPtr(r);
      memcpy(dest_ptr, src_ptr, sizeof(Scalar) * this->cols());
    }
  }

  template <class Scalar_, ImageType ImageType_>  
  void Image_<Scalar_, ImageType_, 1>::fromCv(const cv::Mat& src_) {
    assert(src_.type() == ImageType_);
    MatrixType::resize(src_.rows, src_.cols);
    for (size_t r = 0; r < this->rows(); ++r) {
      const Scalar* src_ptr = src_.ptr<Scalar>(r);
      Scalar* dest_ptr = this->rowPtr(r);
      memcpy(dest_ptr, src_ptr, sizeof(Scalar) * this->cols());
    }
  }


  template <class Scalar_, ImageType ImageType_>    
  void Image_<Scalar_, ImageType_, 1>::fromBuffer(const uint8_t* buffer) {
    const Scalar* src_ptr = reinterpret_cast<const Scalar*>(buffer);
    for (size_t r = 0; r < this->rows(); ++r, src_ptr += cols()) {
      Scalar* dest_ptr = this->rowPtr(r);
      memcpy(dest_ptr, src_ptr, sizeof(Scalar) * this->cols());
    }
  }

  template <class Scalar_, ImageType ImageType_>    
  template <typename DestImageType_, typename ScaleType_>
  void Image_<Scalar_, ImageType_, 1>::convertTo(DestImageType_& dest,
                                                 const ScaleType_& s) const {
    assert(dest.numChannels() == numChannels());
    typedef typename DestImageType_::Scalar DestScalar;
    dest.resize(rows(), cols());
    for (size_t r = 0; r < dest.rows(); ++r) {
      const Scalar* src_ptr = this->rowPtr(r);
      DestScalar* dest_ptr = dest.rowPtr(r);
      for (size_t c = 0; c < cols(); ++c, ++src_ptr, ++dest_ptr) {
        *dest_ptr = (DestScalar)(*src_ptr * s);
      }
    }
  }


  template <class Scalar_, ImageType ImageType_>    
  void Image_<Scalar_, ImageType_, 1>::scale(float scale_factor_) {
    for (size_t r = 0; r < this->rows(); ++r) {
      for (size_t c = 0; c < this->cols(); ++c) {
        (*this)(r, c) *= scale_factor_;
      }
    }
  }
  
  template <class Scalar_, ImageType ImageType_>    
  void Image_<Scalar_, ImageType_, 1>::downsample(Image_<Scalar, ImageType_, 1>& dest,
                                                  DOWNSAMPLE_PIXELS npixels) {
    const float base = sqrt(static_cast<float>(npixels)); //TODO use map between DOWNSAMPLE_PIXELS and its base?
    const float inv_base = 1 / base;
    const float inv_npixels = 1 / (float)npixels;

    int new_rows = this->rows() * inv_base;
    int new_cols = this->cols() * inv_base;

    if (this->rows() % new_rows) {
      throw std::runtime_error("Image::downsample: the rows of dest should divide perfectly the rows of this");
    }
    if (this->cols() % new_cols) {
      throw std::runtime_error("Image::downsample: the cols of dest should divide perfectly the cols of this");
    }

    dest.resize(new_rows, new_cols);
    double tmp = 0;
    int r_idx_start, r_idx_end, c_idx_start, c_idx_end;
    for (int r = 0; r < new_rows; ++r) {
      r_idx_start = r * base;
      r_idx_end = r_idx_start + base;
      for (int c = 0; c < new_cols; ++c) {
        c_idx_start = c * base;
        c_idx_end = c_idx_start + base;
        tmp = 0;
        for (int rr = r_idx_start; rr < r_idx_end; ++rr) {
          for (int cc = c_idx_start; cc < c_idx_end; ++cc) {
            tmp += (*this)(rr, cc);
          }
        }

        dest(r, c) = tmp * inv_npixels;
      }
    }
  }


  
  template <class Scalar_, ImageType ImageType_>    
  Image_<uint8_t,TYPE_8UC1,1> Image_<Scalar_, ImageType_, 1>::operator==(const size_t& value_) const {
    using MaskImage = Image_<uint8_t,TYPE_8UC1,1>;
    MaskImage mask(this->rows(), this->cols());
    
    for (size_t r = 0; r < this->rows(); ++r) {
      const Scalar* src_ptr = this->rowPtr(r);
      uint8_t* dest_ptr = mask.rowPtr(r);
      for (size_t c = 0; c < this->cols(); ++c, ++src_ptr, ++dest_ptr) {
        *dest_ptr = (*src_ptr == value_);
      }
    }
    return mask;
  }
  
  
  
}
