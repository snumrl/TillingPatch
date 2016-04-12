// +-------------------------------------------------------------------------
// | stitch_patch.h
// | 
// | Author: Manmyung Kim
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Manmyung Kim 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the TilingMotionPatch.
// |    TilingMotionPatch is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with TilingMotionPatch.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------
#pragma once

#include "patch.h"

void stitch_patch_half( Patch &pa1, Patch &pa2, int g1, int g2, int p1, int p2, int after_patch_i1, int after_patch_i2 );
void edit_patch_half( Patch &pa1, Patch &pa2, int g1, int g2);
void edit_connected_motion(std::vector<Patch> &patches, int index);