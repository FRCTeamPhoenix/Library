// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.json.simple.parser;

import java.io.IOException;

public interface ContentHandler {
  void startJSON() throws ParseException, IOException;

  void endJSON() throws ParseException, IOException;

  boolean startObject() throws ParseException, IOException;

  boolean endObject() throws ParseException, IOException;

  boolean startObjectEntry(String key) throws ParseException, IOException;

  boolean endObjectEntry() throws ParseException, IOException;

  boolean startArray() throws ParseException, IOException;

  boolean endArray() throws ParseException, IOException;

  boolean primitive(Object value) throws ParseException, IOException;
}
