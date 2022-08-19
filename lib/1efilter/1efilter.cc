/* -*- coding: utf-8 -*-
 *
 * OneEuroFilter.cc -
 *
 * Author: Nicolas Roussel (nicolas.roussel@inria.fr)
 *
 * Copyright 2019 Inria
 *
 * BSD License https://opensource.org/licenses/BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>
#include <cmath>
#include <ctime>

#define M_PI_FLOAT 3.1415926f

// -----------------------------------------------------------------
// Utilities

typedef uint32_t TimeStamp; // in millis

static const TimeStamp UndefinedTime = 0;

// -----------------------------------------------------------------

class LowPassFilter
{

	float y, a, s;
	bool initialized;

	void setAlpha(float alpha)
	{
		if (alpha <= 0.0f || alpha > 1.0f) {
			alpha = 0.1f;
			return;
		}
		a = alpha;
	}

public:
	LowPassFilter(float alpha, float initval = 0.0)
	{
		y = s = initval;
		setAlpha(alpha);
		initialized = false;
	}

	float filter(float value)
	{
		float result;
		if (initialized)
			result = a * value + (1.0f - a) * s;
		else
		{
			result = value;
			initialized = true;
		}
		y = value;
		s = result;
		return result;
	}

	float filterWithAlpha(float value, float alpha)
	{
		setAlpha(alpha);
		return filter(value);
	}

	bool hasLastRawValue(void)
	{
		return initialized;
	}

	float lastRawValue(void)
	{
		return y;
	}
};

// -----------------------------------------------------------------

class OneEuroFilter
{

	float freq;
	float mincutoff;
	float beta_;
	float dcutoff;
	LowPassFilter *x;
	LowPassFilter *dx;
	TimeStamp lasttime;

	float alpha(float cutoff)
	{
		float te = 1.0f / freq;
		float tau = 1.0f / (2 * M_PI_FLOAT  * cutoff);
		return 1.0f / (1.0f + tau / te);
	}

	void setFrequency(float f)
	{
		if (f <= 0) {
			f = 0.1f;
			return;
		}
		freq = f;
	}

	void setMinCutoff(float mc)
	{
		if (mc <= 0) {
			mc = 0.1f;
			return;
		}
		mincutoff = mc;
	}

	void setBeta(float b)
	{
		beta_ = b;
	}

	void setDerivateCutoff(float dc)
	{
		if (dc <= 0) {
			dc = 0.1f;
			return;
		}
		dcutoff = dc;
	}

public:
	OneEuroFilter(float freq,
				  float mincutoff = 1.0f, float beta_ = 0.0, float dcutoff = 1.0f)
	{
		setFrequency(freq);
		setMinCutoff(mincutoff);
		setBeta(beta_);
		setDerivateCutoff(dcutoff);
		x = new LowPassFilter(alpha(mincutoff));
		dx = new LowPassFilter(alpha(dcutoff));
		lasttime = UndefinedTime;
	}

	float filter(float value, uint32_t timestamp = UndefinedTime)
	{
		// update the sampling frequency based on timestamps
		if (lasttime != UndefinedTime && timestamp != UndefinedTime)
			freq = 1000000.f / (timestamp - lasttime);
		lasttime = timestamp;
		// estimate the current variation per second
		float dvalue = x->hasLastRawValue() ? (value - x->lastRawValue()) * freq : 0.0; // FIXME: 0.0 or value?
		float edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff));
		// use it to update the cutoff frequency
		float cutoff = mincutoff + beta_ * fabs(edvalue);
		// filter the given value
		return x->filterWithAlpha(value, alpha(cutoff));
	}

	~OneEuroFilter(void)
	{
		delete x;
		delete dx;
	}
};