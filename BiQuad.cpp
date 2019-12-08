/*
    This file is part of EqualizerAPO, a system-wide equalizer.
    Copyright (C) 2013  Jonas Thedering

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/
/*
30/11/2019 file was changed in order to use it for Teensy audio
*/


#include "BiQuad.h"


BiQuad::BiQuad(Type type, double dbGain, double freq, double srate, double bandwidthOrQOrS, bool isBandwidthOrS)
{
	const double A =(type == PEAKING || type == LOW_SHELF || type == HIGH_SHELF) ? pow(10, dbGain / 40) : pow(10, dbGain / 20);
	
	const double omega = 2 * M_PI * freq / srate;
	const double sn = sin(omega);
	const double cs = cos(omega);
	double alpha;

	if (!isBandwidthOrS) // Q
		alpha = sn / (2 * bandwidthOrQOrS);
	else if (type == LOW_SHELF || type == HIGH_SHELF) // S
		alpha = sn / 2 * sqrt((A + 1 / A) * (1 / bandwidthOrQOrS - 1) + 2);
	else // BW
		alpha = sn * sinh(_M_LN2 / 2 * bandwidthOrQOrS * omega / sn);

	const double beta = 2 * sqrt(A) * alpha;

	double b0=1., b1=1., b2=1., a0=1., a1=1., a2=1.;

	switch (type)
	{
	case LOW_PASS:
		b0 = (1 - cs) / 2;
		b1 = 1 - cs;
		b2 = (1 - cs) / 2;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case HIGH_PASS:
		b0 = (1 + cs) / 2;
		b1 = -(1 + cs);
		b2 = (1 + cs) / 2;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case BAND_PASS:
		b0 = alpha;
		b1 = 0;
		b2 = -alpha;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case NOTCH:
		b0 = 1;
		b1 = -2 * cs;
		b2 = 1;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case ALL_PASS:
		b0 = 1 - alpha;
		b1 = -2 * cs;
		b2 = 1 + alpha;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case PEAKING:
		b0 = 1 + (alpha * A);
		b1 = -2 * cs;
		b2 = 1 - (alpha * A);
		a0 = 1 + (alpha / A);
		a1 = -2 * cs;
		a2 = 1 - (alpha / A);
		break;
	case LOW_SHELF:
		b0 = A * ((A + 1) - (A - 1) * cs + beta);
		b1 = 2 * A * ((A - 1) - (A + 1) * cs);
		b2 = A * ((A + 1) - (A - 1) * cs - beta);
		a0 = (A + 1) + (A - 1) * cs + beta;
		a1 = -2 * ((A - 1) + (A + 1) * cs);
		a2 = (A + 1) + (A - 1) * cs - beta;
		break;
	case HIGH_SHELF:
		b0 = A * ((A + 1) + (A - 1) * cs + beta);
		b1 = -2 * A * ((A - 1) + (A + 1) * cs);
		b2 = A * ((A + 1) + (A - 1) * cs - beta);
		a0 = (A + 1) - (A - 1) * cs + beta;
		a1 = 2 * ((A - 1) - (A + 1) * cs);
		a2 = (A + 1) - (A - 1) * cs - beta;
		break;
	}

	this->a0 = b0 / a0;
	this->a[0] = b1 / a0;
	this->a[1] = b2 / a0;
	this->a[2] = a1 / a0;
	this->a[3] = a2 / a0;

	x1 = 0;
	x2 = 0;
	y1 = 0;
	y2 = 0;
}
void BiQuad::reset(){
	x1 = 0;
	x2 = 0;
	y1 = 0;
	y2 = 0;
}