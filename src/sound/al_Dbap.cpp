#include "al/sound/al_Dbap.hpp"

#include <array>
#include <stdexcept>

namespace al {

Dbap::Dbap(const Speakers &sl, float focus)
    : Spatializer(sl),
      mNumSpeakers(0),
      mFocus((focus >= 0.1f) ? focus : 0.1f) {
  mNumSpeakers = mSpeakers.size();
  if (mNumSpeakers > DBAP_MAX_NUM_SPEAKERS) {
    throw std::runtime_error(
        "Dbap: speaker count " + std::to_string(mNumSpeakers) +
        " exceeds DBAP_MAX_NUM_SPEAKERS (" +
        std::to_string(DBAP_MAX_NUM_SPEAKERS) + ")");
  }
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
  std::array<float, DBAP_MAX_NUM_SPEAKERS> w;
  for (unsigned int i = 0; i < mNumSpeakers; ++i) {
    Vec3d vec = relpos - mSpeakerVecs[i];
    double dist = vec.mag();
    w[i] = powf(1.f / (1.f + float(dist)), mFocus);
  }

  // Step 2: max-scaled L2 normalization (Lucian Parisi)
  // Weights are divided by maxW before squaring so sumSq is always in [1, N].
  // This avoids large intermediate normalizer factors when all raw weights are
  // very small (extreme focus + distant speakers). Guard is on maxW directly:
  // if the loudest raw weight is negligible, output silence.
  float maxW = 0.0f;
  for (unsigned int i = 0; i < mNumSpeakers; ++i) {
    if (w[i] > maxW) maxW = w[i];
  }
  if (maxW < 1e-6f) return;

  float sumSq = 0.0f;
  for (unsigned int i = 0; i < mNumSpeakers; ++i) {
    float ws = w[i] / maxW;
    sumSq += ws * ws;
  }
  float kNorm = 1.0f / (maxW * sqrtf(sumSq));

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
  std::array<float, DBAP_MAX_NUM_SPEAKERS> w;
  for (unsigned int k = 0; k < mNumSpeakers; ++k) {
    Vec3d vec = relpos - mSpeakerVecs[k];
    double dist = vec.mag();
    w[k] = powf(1.0f / (1.0f + float(dist)), mFocus);
  }

  // Step 2: max-scaled L2 normalization (Lucian Parisi)
  // Weights are divided by maxW before squaring so sumSq is always in [1, N].
  // This avoids large intermediate normalizer factors when all raw weights are
  // very small (extreme focus + distant speakers). Guard is on maxW directly:
  // if the loudest raw weight is negligible, output silence.
  float maxW = 0.0f;
  for (unsigned int k = 0; k < mNumSpeakers; ++k) {
    if (w[k] > maxW) maxW = w[k];
  }
  if (maxW < 1e-6f) return;

  float sumSq = 0.0f;
  for (unsigned int k = 0; k < mNumSpeakers; ++k) {
    float ws = w[k] / maxW;
    sumSq += ws * ws;
  }
  float kNorm = 1.0f / (maxW * sqrtf(sumSq));

  // Precompute final per-speaker gains outside the sample loop (Lucian Parisi)
  std::array<float, DBAP_MAX_NUM_SPEAKERS> gain;
  for (unsigned int k = 0; k < mNumSpeakers; ++k) gain[k] = kNorm * w[k];

  // Step 3: apply normalized gains to output (Lucian Parisi)
  for (unsigned int k = 0; k < mNumSpeakers; ++k) {
    float *out = io.outBuffer(mDeviceChannels[k]);
    for (size_t i = 0; i < numFrames; ++i) {
      out[i] += gain[k] * samples[i];
    }
  }
}

void Dbap::print(std::ostream &stream) {
  stream << "Using DBAP Panning- need to add panner info for print function"
         << std::endl;
}

} // namespace al
