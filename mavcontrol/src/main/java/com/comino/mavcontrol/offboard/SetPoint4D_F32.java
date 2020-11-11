/****************************************************************************
 *
 *   Copyright (c) 2017,2020 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

package com.comino.mavcontrol.offboard;



import com.comino.mavcom.struct.Polar4D_F32;

import georegression.struct.point.Point4D_F32;
import georegression.struct.point.Vector3D_F32;
import georegression.struct.point.Vector4D_F32;

public class SetPoint4D_F32 extends Vector4D_F32 {

	private static final long serialVersionUID = 5387076120108437275L;
	private long  delta_tms = 0;

	public SetPoint4D_F32() {
		super(); clear();
	}

	public SetPoint4D_F32(float x, float y, float z, float w) {
		super(x, y, z, w);
		delta_tms = -1;
	}

	public SetPoint4D_F32(Point4D_F32 a, Point4D_F32 b) {
		super(a, b);
		delta_tms = -1;
	}

	public void set(Vector3D_F32 s, float w) {
		this.set(s.x, s.y, s.z, w);
		delta_tms = -1;
	}

	public void set(Vector3D_F32 s, float w, long delta_ms) {
		this.set(s.x, s.y, s.z, w);
		delta_tms = delta_ms;
	}

	public long getTimestampDelta() {
		return delta_tms;
	}

	public long getPolar(Polar4D_F32 p) {
		p.set(this);
		return delta_tms;
	}

	public void clear() {
		this.set(Float.NaN, Float.NaN, Float.NaN, Float.NaN);
		this.delta_tms = -1;
	}


}
