#include "al/sound/al_Dbap.hpp"

namespace al {

Dbap::Dbap(const Speakers &sl, float focus)
    : Spatializer(sl), mNumSpeakers(0), mFocus(focus) {
  mNumSpeakers = mSpeakers.size();
  std::cout << "DBAP Compiled with " << mNumSpeakers << " speakers"
            << std::endl;

  for (unsigned int i = 0; i < mNumSpeakers; i++) {
    mSpeakerVecs[i] = mSpeakers[i].vec();
    mDeviceChannels[i] = mSpeakers[i].deviceChannel;
  }
}

void Dbap::renderSample(AudioIOData &io, const Vec3f &pos, const float &sample,
                        const unsigned int &frameIndex) {
  //  Vec3d relpos = listeningPose.vec();

  //  // Rotate vector according to listener-rotation
  //  Quatd srcRot = listeningPose.quat();
  //  relpos = srcRot.rotate(relpos);
  //  relpos = Vec3d(relpos.x, relpos.z, relpos.y);

  Vec3d relpos = Vec3d(pos.x, -pos.z, pos.y);

  // Original AlloLib behavior (Ryan McGee): raw exponent on per-speaker
  //   inverse-distance gain, no normalization — total power decreases as
  //   focus increases.
  // Cult DSP modification (Lucian Parisi): L2 normalization added so that
  //   sum(v_k^2) = 1 for all source positions and focus values.
  //   This matches equation (2) of Lossius et al., ICMC 2009.

  // Step 1: compute unnormalized weights (original per-speaker distance loop)
  float w[DBAP_MAX_NUM_SPEAKERS];
  for (unsigned int i = 0; i < mNumSpeakers; ++i) {
    Vec3d vec = relpos - mSpeakerVecs[i];
    double dist = vec.mag();
    w[i] = powf(1.f / (1.f + float(dist)), mFocus);
  }

  // Step 2: L2 normalizer (Lucian Parisi)
  float sumSq = 0.0f;
  for (unsigned int i = 0; i < mNumSpeakers; ++i) sumSq += w[i] * w[i];
  float kNorm = (sumSq > 1e-12f) ? (1.0f / sqrtf(sumSq)) : 0.0f;

  // Step 3: apply normalized gain to output (Lucian Parisi)
  for (unsigned int i = 0; i < mNumSpeakers; ++i) {
    io.out(mDeviceChannels[i], frameIndex) += kNorm * w[i] * sample;
  }
}

void Dbap::renderBuffer(AudioIOData &io, const Vec3f &pos, const float *samples,
                        const unsigned int &numFrames) {
  //  Vec3d relpos = listeningPose.vec();

  //  // Rotate vector according to listener-rotation
  //  Quatd srcRot = listeningPose.quat();
  //  relpos = srcRot.rotate(relpos);

  // FIXME test DBAP
  Vec3d relpos = Vec3d(pos.x, -pos.z, pos.y);

  // Original AlloLib behavior (Ryan McGee): raw exponent on per-speaker
  //   inverse-distance gain, no normalization — total power decreases as
  //   focus increases.
  // Cult DSP modification (Lucian Parisi): L2 normalization added so that
  //   sum(v_k^2) = 1 for all source positions and focus values.
  //   This matches equation (2) of Lossius et al., ICMC 2009.

  // Step 1: compute unnormalized weights (original per-speaker distance loop)
  float w[DBAP_MAX_NUM_SPEAKERS];
  for (unsigned int k = 0; k < mNumSpeakers; ++k) {
    Vec3d vec = relpos - mSpeakerVecs[k];
    double dist = vec.mag();
    w[k] = powf(1.0f / (1.0f + float(dist)), mFocus);
  }

  // Step 2: L2 normalizer (Lucian Parisi)
  float sumSq = 0.0f;
  for (unsigned int k = 0; k < mNumSpeakers; ++k) sumSq += w[k] * w[k];
  float kNorm = (sumSq > 1e-12f) ? (1.0f / sqrtf(sumSq)) : 0.0f;

  // Step 3: apply normalized gain to output (Lucian Parisi)
  for (unsigned int k = 0; k < mNumSpeakers; ++k) {
    float *out = io.outBuffer(mDeviceChannels[k]);
    for (size_t i = 0; i < numFrames; ++i) {
      out[i] += kNorm * w[k] * samples[i];
    }
  }
}

void Dbap::print(std::ostream &stream) {
  stream << "Using DBAP Panning- need to add panner info for print function"
         << std::endl;
}

} // namespace al
