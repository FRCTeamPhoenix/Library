// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.json.simple;

import java.io.IOException;
import java.io.Writer;

public interface JSONStreamAware {
  void writeJSONString(Writer out) throws IOException;
}
