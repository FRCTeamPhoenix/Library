// Copyright (c) 2026 Team 2342
// https://github.com/FRCTeamPhoenix
//
// This source code is licensed under the MIT License.
// See the LICENSE file in the root directory of this project.

package org.json.simple;

import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

public class JSONArray extends ArrayList implements JSONAware, JSONStreamAware {
  private static final long serialVersionUID = 3957988303675231981L;

  public JSONArray() {
    super();
  }

  public JSONArray(Collection c) {
    super(c);
  }

  public static void writeJSONString(Collection collection, Writer out) throws IOException {
    if (collection == null) {
      out.write("null");
      return;
    }

    boolean first = true;
    Iterator iter = collection.iterator();

    out.write('[');
    while (iter.hasNext()) {
      if (first) first = false;
      else out.write(',');

      Object value = iter.next();
      if (value == null) {
        out.write("null");
        continue;
      }

      JSONValue.writeJSONString(value, out);
    }
    out.write(']');
  }

  public void writeJSONString(Writer out) throws IOException {
    writeJSONString(this, out);
  }

  public static String toJSONString(Collection collection) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(collection, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(byte[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[");
      out.write(String.valueOf(array[0]));

      for (int i = 1; i < array.length; i++) {
        out.write(",");
        out.write(String.valueOf(array[i]));
      }

      out.write("]");
    }
  }

  public static String toJSONString(byte[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(short[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[");
      out.write(String.valueOf(array[0]));

      for (int i = 1; i < array.length; i++) {
        out.write(",");
        out.write(String.valueOf(array[i]));
      }

      out.write("]");
    }
  }

  public static String toJSONString(short[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(int[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[");
      out.write(String.valueOf(array[0]));

      for (int i = 1; i < array.length; i++) {
        out.write(",");
        out.write(String.valueOf(array[i]));
      }

      out.write("]");
    }
  }

  public static String toJSONString(int[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(long[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[");
      out.write(String.valueOf(array[0]));

      for (int i = 1; i < array.length; i++) {
        out.write(",");
        out.write(String.valueOf(array[i]));
      }

      out.write("]");
    }
  }

  public static String toJSONString(long[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(float[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[");
      out.write(String.valueOf(array[0]));

      for (int i = 1; i < array.length; i++) {
        out.write(",");
        out.write(String.valueOf(array[i]));
      }

      out.write("]");
    }
  }

  public static String toJSONString(float[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(double[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[");
      out.write(String.valueOf(array[0]));

      for (int i = 1; i < array.length; i++) {
        out.write(",");
        out.write(String.valueOf(array[i]));
      }

      out.write("]");
    }
  }

  public static String toJSONString(double[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(boolean[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[");
      out.write(String.valueOf(array[0]));

      for (int i = 1; i < array.length; i++) {
        out.write(",");
        out.write(String.valueOf(array[i]));
      }

      out.write("]");
    }
  }

  public static String toJSONString(boolean[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(char[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[\"");
      out.write(String.valueOf(array[0]));

      for (int i = 1; i < array.length; i++) {
        out.write("\",\"");
        out.write(String.valueOf(array[i]));
      }

      out.write("\"]");
    }
  }

  public static String toJSONString(char[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public static void writeJSONString(Object[] array, Writer out) throws IOException {
    if (array == null) {
      out.write("null");
    } else if (array.length == 0) {
      out.write("[]");
    } else {
      out.write("[");
      JSONValue.writeJSONString(array[0], out);

      for (int i = 1; i < array.length; i++) {
        out.write(",");
        JSONValue.writeJSONString(array[i], out);
      }

      out.write("]");
    }
  }

  public static String toJSONString(Object[] array) {
    final StringWriter writer = new StringWriter();

    try {
      writeJSONString(array, writer);
      return writer.toString();
    } catch (IOException e) {
      // This should never happen for a StringWriter
      throw new RuntimeException(e);
    }
  }

  public String toJSONString() {
    return toJSONString(this);
  }

  public String toString() {
    return toJSONString();
  }
}
