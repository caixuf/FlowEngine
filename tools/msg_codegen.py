#!/usr/bin/env python3
"""
FlowEngine IDL Code Generator (Phase 1)

Parses .msg IDL files and generates C headers with:
  - C struct definitions
  - Compile-time type IDs (FNV-1a hash)
  - Schema version constants
  - Endian-aware serialize/deserialize functions
  - Runtime type registration

Usage:
  python3 tools/msg_codegen.py msg/adas_msgs.msg build/gen/adas_msgs_gen.h

IDL Syntax:
  # comments
  enum Name { VALUE = int, ... }
  struct Name { type field [= default]; type field[N]; ... }

Supported types:
  bool, int8, uint8, int16, uint16, int32, uint32,
  int64, uint64, float, float64, enum types, nested structs
"""

import sys
import re
import os
from dataclasses import dataclass, field
from typing import Optional

# =============================================================================
# FNV-1a Hash (32-bit, same as runtime in serializer.c)
# =============================================================================

def fnv1a_hash(s: str) -> int:
    """Compute FNV-1a 32-bit hash of a string."""
    h = 0x811c9dc5
    for b in s.encode('utf-8'):
        h = ((h ^ b) * 0x01000193) & 0xffffffff
    return h

# =============================================================================
# Type system
# =============================================================================

PRIMITIVE_SIZES = {
    'bool':    1,  # uint8 in serialization
    'int8':    1, 'uint8':   1,
    'int16':   2, 'uint16':  2,
    'int32':   4, 'uint32':  4,
    'int64':   8, 'uint64':  8,
    'float':   4,            # IEEE 754 single
    'float64': 8,            # IEEE 754 double
    'char':    1,            # single byte (arrays = fixed strings)
}

PRIMITIVE_C_TYPES = {
    'bool':    'bool',
    'int8':    'int8_t',
    'uint8':   'uint8_t',
    'int16':   'int16_t',
    'uint16':  'uint16_t',
    'int32':   'int32_t',
    'uint32':  'uint32_t',
    'int64':   'int64_t',
    'uint64':  'uint64_t',
    'float':   'float',
    'float64': 'double',
}

def is_integer_type(t: str) -> bool:
    return t in ('bool', 'int8', 'uint8', 'int16', 'uint16',
                 'int32', 'uint32', 'int64', 'uint64')

def is_float_type(t: str) -> bool:
    return t in ('float', 'float64')

# =============================================================================
# IDL AST
# =============================================================================

@dataclass
class Field:
    name: str
    c_type: str       # C type name (e.g., "float", "Obstacle")
    idl_type: str     # original IDL type (e.g., "float", "Obstacle")
    array_size: int   # 0 = not array, >0 = fixed array
    is_enum: bool = False
    is_nested: bool = False
    nested_typename: str = ""  # original struct typename if nested

@dataclass
class EnumDef:
    name: str
    values: list  # list of (name, value)

@dataclass
class StructDef:
    name: str
    fields: list  # list of Field
    depends_on: list = field(default_factory=list)  # names of structs/enums this depends on

# =============================================================================
# Parser
# =============================================================================

class ParseError(Exception):
    def __init__(self, msg, line_no=None, line=""):
        loc = f" line {line_no}: {line.strip()}" if line_no else ""
        super().__init__(f"Parse error{loc}: {msg}")

class IDLParser:
    def __init__(self, text: str):
        self.lines = text.split('\n')
        self.pos = 0
        self.enums: dict[str, EnumDef] = {}
        self.structs: dict[str, StructDef] = {}

    def peek(self) -> Optional[str]:
        while self.pos < len(self.lines):
            line = self._strip_comment(self.lines[self.pos]).strip()
            if line:
                return line
            self.pos += 1
        return None

    def next_line(self) -> str:
        line = self.peek()
        if line is None:
            raise ParseError("Unexpected end of file")
        self.pos += 1
        return line

    def _strip_comment(self, line: str) -> str:
        idx = line.find('#')
        return line[:idx] if idx >= 0 else line

    def parse(self):
        while self.peek() is not None:
            line = self.peek()
            if line.startswith('enum '):
                self._parse_enum()
            elif line.startswith('struct '):
                self._parse_struct()
            else:
                raise ParseError(f"Unexpected token: {line}", self.pos + 1, self.lines[self.pos])
        return self

    def _parse_enum(self):
        line = self.next_line()
        m = re.match(r'^enum\s+(\w+)\s*\{', line)
        if not m:
            raise ParseError(f"Invalid enum declaration", self.pos, line)
        name = m.group(1)
        values = []
        while True:
            inner = self.next_line()
            if inner.strip() == '}':
                break
            m2 = re.match(r'\s*(\w+)\s*=\s*(-?\d+)', inner)
            if not m2:
                raise ParseError(f"Invalid enum value", self.pos, inner)
            values.append((m2.group(1), int(m2.group(2))))
        self.enums[name] = EnumDef(name, values)

    def _parse_struct(self):
        line = self.next_line()
        m = re.match(r'^struct\s+(\w+)\s*\{', line)
        if not m:
            raise ParseError(f"Invalid struct declaration", self.pos, line)
        name = m.group(1)
        fields = []
        while True:
            inner = self.next_line()
            if inner.strip() == '}':
                break
            fields.append(self._parse_field(inner))

        # Compute dependencies: any field whose type is a struct/enum in our set
        deps = []
        for f in fields:
            base_type = f.idl_type
            # Already known? Will be resolved later in topological sort
            if base_type in self.structs:
                deps.append(base_type)
            elif base_type in self.enums:
                deps.append(base_type)

        self.structs[name] = StructDef(name, fields, deps)

    def _parse_field(self, line: str) -> Field:
        line = line.strip()
        # Pattern: type name  OR  type name[N]  OR  type name = default
        m = re.match(r'(\w+)\s+(\w+)(?:\[(\d+)\])?\s*(?:=\s*(.+))?$', line)
        if not m:
            raise ParseError(f"Invalid field: {line}")
        idl_type = m.group(1)
        field_name = m.group(2)
        array_size = int(m.group(3)) if m.group(3) else 0
        # default_val = m.group(4)  # reserved for future

        # Determine C type
        is_enum = idl_type in self.enums
        is_nested = idl_type in self.structs
        c_type = idl_type
        if idl_type in PRIMITIVE_C_TYPES:
            c_type = PRIMITIVE_C_TYPES[idl_type]
        elif is_enum:
            c_type = 'int8_t'  # enums are serialized as int8

        return Field(
            name=field_name,
            c_type=c_type,
            idl_type=idl_type,
            array_size=array_size,
            is_enum=is_enum,
            is_nested=is_nested,
            nested_typename=idl_type if is_nested else ""
        )

# =============================================================================
# Code Generator
# =============================================================================

class CodeGenerator:
    def __init__(self, parser: IDLParser):
        self.enums = parser.enums
        self.structs = parser.structs
        self._sort_structs()

    def _sort_structs(self):
        """Topological sort structs by dependency."""
        ordered = []
        visited = set()
        temp = set()

        def visit(name):
            if name in visited:
                return
            if name in temp:
                # Cyclic dependency — warn and proceed
                print(f"Warning: cyclic dependency involving struct '{name}'", file=sys.stderr)
                return
            temp.add(name)
            for dep in self.structs[name].depends_on:
                if dep in self.structs:
                    visit(dep)
            temp.discard(name)
            visited.add(name)
            ordered.append(name)

        for name in self.structs:
            if name not in visited:
                visit(name)

        self._struct_order = ordered

    def _sig_string(self, s: StructDef) -> str:
        """Build type signature string for FNV-1a hashing."""
        parts = []
        for f in s.fields:
            if f.is_nested:
                # Include nested struct's full signature
                nested = self.structs.get(f.nested_typename)
                if nested:
                    inner = self._sig_string(nested)
                    parts.append(f"{inner}{f.array_size > 0 and f'[{f.array_size}]' or ''}")
                else:
                    parts.append(f"{f.idl_type}_{f.name}")
            else:
                parts.append(f"{f.idl_type}_{f.name}")
        return f"{s.name}:" + ",".join(parts)

    def _type_id(self, s: StructDef) -> int:
        return fnv1a_hash(self._sig_string(s))

    def _layout_string(self, s: StructDef) -> str:
        """Field-level layout signature for schema_hash (name+type+array)."""
        parts = []
        for f in s.fields:
            arr = f.array_size if f.array_size > 0 else 1
            parts.append(f"{f.name}:{f.idl_type}:{arr}")
        return f"{s.name}#" + ";".join(parts)

    def _schema_hash(self, s: StructDef) -> int:
        return fnv1a_hash(self._layout_string(s))

    def _field_kind(self, f: Field) -> str:
        """Map an IDL field to a FieldKind enum constant name."""
        if f.is_nested:
            return "FIELD_KIND_NESTED"
        if f.is_enum:
            return "FIELD_KIND_ENUM"
        if f.idl_type == 'bool':
            return "FIELD_KIND_BOOL"
        if f.idl_type in ('float', 'float64'):
            return "FIELD_KIND_FLOAT"
        if f.idl_type in ('int8', 'int16', 'int32', 'int64'):
            return "FIELD_KIND_INT"
        if f.idl_type in ('uint8', 'uint16', 'uint32', 'uint64'):
            return "FIELD_KIND_UINT"
        return "FIELD_KIND_UNKNOWN"

    def _field_size(self, f: Field) -> int:
        """Get the serialized size of a single field element."""
        if f.idl_type in PRIMITIVE_SIZES:
            return PRIMITIVE_SIZES[f.idl_type]
        if f.is_enum:
            return 1  # enum = int8
        if f.is_nested:
            nested = self.structs.get(f.nested_typename)
            if nested:
                return self._struct_size(nested)
        return 0  # unknown

    def _struct_size(self, s: StructDef) -> int:
        total = 0
        for f in s.fields:
            elem_size = self._field_size(f)
            count = f.array_size if f.array_size > 0 else 1
            total += elem_size * count
        return total

    def _struct_deps(self, s: StructDef) -> tuple[list[str], list[str]]:
        """Return (enum_deps, struct_deps) for a given struct."""
        enum_deps = []
        struct_deps = []
        for f in s.fields:
            if f.is_enum and f.idl_type not in enum_deps:
                enum_deps.append(f.idl_type)
            if f.is_nested and f.idl_type not in struct_deps:
                struct_deps.append(f.idl_type)
        return enum_deps, struct_deps

    def generate(self) -> str:
        lines = []
        lines.append("/*")
        lines.append(" * AUTO-GENERATED by FlowEngine msg_codegen.py — DO NOT EDIT")
        lines.append(" * Source: msg/adas_msgs.msg")
        lines.append(" */")
        lines.append("")
        lines.append("#ifndef ADAS_MSGS_GEN_H")
        lines.append("#define ADAS_MSGS_GEN_H")
        lines.append("")
        lines.append("#include <stdint.h>")
        lines.append("#include <stdbool.h>")
        lines.append("#include <string.h>")
        lines.append("#include <stddef.h>")
        lines.append("#include \"serializer.h\"")
        lines.append("")
        lines.append("#ifdef __cplusplus")
        lines.append("extern \"C\" {")
        lines.append("#endif")
        lines.append("")

        # Generate enums
        for name, edef in self.enums.items():
            lines.extend(self._gen_enum(edef))

        # Generate structs in dependency order
        for name in self._struct_order:
            s = self.structs[name]
            lines.extend(self._gen_struct(s))

        # Generate registration helper
        lines.extend(self._gen_registration())

        lines.append("")
        lines.append("#ifdef __cplusplus")
        lines.append("}")
        lines.append("#endif")
        lines.append("")
        lines.append("#endif /* ADAS_MSGS_GEN_H */")

        return '\n'.join(lines)

    def generate_split(self, output_dir: str, msg_basename: str) -> list[str]:
        """Generate per-struct headers under output_dir/gen/.

        Returns list of generated file paths (relative to output_dir).
        Layout:
          gen/adas_enums.h          — shared enum definitions
          gen/<StructName>.h        — per-struct header (includes its deps)
          gen/adas_msgs_register_all.h — batch registration
          gen/adas_msgs_gen.h       — umbrella header (backward compat)
        """
        import os
        gen_dir = os.path.join(output_dir, "gen")
        os.makedirs(gen_dir, exist_ok=True)

        generated = []

        # 1. Shared enums header
        enums_path = os.path.join(gen_dir, "adas_enums.h")
        self._write_file(enums_path, self._gen_split_enums_header())
        generated.append("gen/adas_enums.h")

        # 2. Per-struct headers
        for name in self._struct_order:
            s = self.structs[name]
            struct_path = os.path.join(gen_dir, f"{name}.h")
            self._write_file(struct_path, self._gen_split_struct_header(s))
            generated.append(f"gen/{name}.h")

        # 3. Batch registration header
        reg_path = os.path.join(gen_dir, "adas_msgs_register_all.h")
        self._write_file(reg_path, self._gen_split_register_all_header())
        generated.append("gen/adas_msgs_register_all.h")

        # 4. Umbrella header (backward compat)
        umbrella_path = os.path.join(gen_dir, f"{msg_basename}_gen.h")
        self._write_file(umbrella_path, self._gen_split_umbrella_header(msg_basename))
        generated.append(f"gen/{msg_basename}_gen.h")

        return generated

    def _write_file(self, path: str, content: str):
        with open(path, 'w') as f:
            f.write(content)

    def _gen_split_enums_header(self) -> str:
        lines = []
        lines.append("/*")
        lines.append(" * AUTO-GENERATED by FlowEngine msg_codegen.py — DO NOT EDIT")
        lines.append(" * Source: msg/adas_msgs.msg")
        lines.append(" * Shared enum definitions for ADAS message types")
        lines.append(" */")
        lines.append("")
        lines.append("#ifndef ADAS_ENUMS_H")
        lines.append("#define ADAS_ENUMS_H")
        lines.append("")
        lines.append("#include <stdint.h>")
        lines.append("")

        for name, edef in self.enums.items():
            lines.extend(self._gen_enum(edef))

        lines.append("#endif /* ADAS_ENUMS_H */")
        lines.append("")
        return '\n'.join(lines)

    def _gen_split_struct_header(self, s: StructDef) -> str:
        """Generate a self-contained header for a single struct with all its deps."""
        enum_deps, struct_deps = self._struct_deps(s)
        type_id = self._type_id(s)
        struct_size = self._struct_size(s)
        guard = f"{s.name.upper()}_GEN_H"

        lines = []
        lines.append("/*")
        lines.append(" * AUTO-GENERATED by FlowEngine msg_codegen.py — DO NOT EDIT")
        lines.append(f" * Struct: {s.name}")
        lines.append(" */")
        lines.append("")
        lines.append(f"#ifndef {guard}")
        lines.append(f"#define {guard}")
        lines.append("")
        lines.append("#include <stdint.h>")
        lines.append("#include <stdbool.h>")
        lines.append("#include <string.h>")
        lines.append("#include <stddef.h>")
        lines.append("#include \"serializer.h\"")

        # Include shared enums if needed
        if enum_deps:
            lines.append("#include \"adas_enums.h\"")

        # Include dependent struct headers
        for dep in struct_deps:
            lines.append(f"#include \"{dep}.h\"")

        lines.append("")
        lines.append("#ifdef __cplusplus")
        lines.append("extern \"C\" {")
        lines.append("#endif")
        lines.append("")

        # Generate this struct
        lines.extend(self._gen_struct(s))

        lines.append("#ifdef __cplusplus")
        lines.append("}")
        lines.append("#endif")
        lines.append("")
        lines.append(f"#endif /* {guard} */")
        lines.append("")
        return '\n'.join(lines)

    def _gen_split_register_all_header(self) -> str:
        """Generate header with adas_msgs_register_all() that includes all structs."""
        lines = []
        lines.append("/*")
        lines.append(" * AUTO-GENERATED by FlowEngine msg_codegen.py — DO NOT EDIT")
        lines.append(" * Batch registration for all ADAS message types")
        lines.append(" */")
        lines.append("")
        lines.append("#ifndef ADAS_MSGS_REGISTER_ALL_H")
        lines.append("#define ADAS_MSGS_REGISTER_ALL_H")
        lines.append("")

        # Include all struct headers
        for name in self._struct_order:
            lines.append(f"#include \"{name}.h\"")

        lines.append("")
        lines.append("#ifdef __cplusplus")
        lines.append("extern \"C\" {")
        lines.append("#endif")
        lines.append("")

        lines.extend(self._gen_registration())

        lines.append("#ifdef __cplusplus")
        lines.append("}")
        lines.append("#endif")
        lines.append("")
        lines.append("#endif /* ADAS_MSGS_REGISTER_ALL_H */")
        lines.append("")
        return '\n'.join(lines)

    def _gen_split_umbrella_header(self, msg_basename: str) -> str:
        """Generate umbrella header that includes everything (backward compat)."""
        guard = f"{msg_basename.upper()}_GEN_H"
        lines = []
        lines.append("/*")
        lines.append(" * AUTO-GENERATED by FlowEngine msg_codegen.py — DO NOT EDIT")
        lines.append(" * Umbrella header — includes all ADAS message type headers.")
        lines.append(" * For smaller compile times, include only the specific headers you need:")
        lines.append(f" *   #include \"LidarFrame.h\"")
        lines.append(f" *   #include \"ObstacleList.h\"")
        lines.append(" */")
        lines.append("")
        lines.append(f"#ifndef {guard}")
        lines.append(f"#define {guard}")
        lines.append("")
        lines.append("#include \"adas_enums.h\"")
        for name in self._struct_order:
            lines.append(f"#include \"{name}.h\"")
        lines.append("#include \"adas_msgs_register_all.h\"")
        lines.append("")
        lines.append(f"#endif /* {guard} */")
        lines.append("")
        return '\n'.join(lines)

    def _gen_enum(self, e: EnumDef) -> list[str]:
        lines = []
        lines.append(f"/* Enum: {e.name} */")
        lines.append(f"typedef enum {{")
        for vname, vval in e.values:
            lines.append(f"    {vname} = {vval},")
        lines.append(f"}} {e.name};")
        lines.append("")
        return lines

    def _gen_struct(self, s: StructDef) -> list[str]:
        type_id = self._type_id(s)
        struct_size = self._struct_size(s)
        sig = self._sig_string(s)

        lines = []
        lines.append("/* ──────────────────────────────────────────────────────── */")
        lines.append(f"/* Struct: {s.name} (size={struct_size}B, type_id=0x{type_id:08x}) */")
        lines.append(f"/* Sig: {sig[:80]}... */")
        lines.append("")

        # Type ID and schema version
        lines.append(f"#define {s.name.upper()}_TYPE_ID         0x{type_id:08x}u")
        lines.append(f"#define {s.name.upper()}_SCHEMA_VERSION  1")
        lines.append(f"#define {s.name.upper()}_SCHEMA_HASH     0x{self._schema_hash(s):08x}u")
        lines.append(f"#define {s.name.upper()}_TYPE_NAME        \"{s.name}\"")
        lines.append("")

        # C struct definition
        lines.append(f"typedef struct {{")
        for f in s.fields:
            array_suffix = f"[{f.array_size}]" if f.array_size > 0 else ""
            comment = f"  /**< {f.idl_type} */"
            lines.append(f"    {f.c_type}    {f.name}{array_suffix};{comment}")
        lines.append(f"}} {s.name};")
        lines.append("")

        # C++ type traits
        lines.append("#ifdef __cplusplus")
        lines.append("}  /* close extern \"C\" — template specialization needs C++ linkage */")
        lines.append(f"/* C++ type traits for msg_cast<{s.name}>(msg) */")
        lines.append(f"template<> struct msg_traits<{s.name}> {{")
        lines.append(f"    static constexpr uint32_t TYPE_ID = {s.name.upper()}_TYPE_ID;")
        lines.append(f"    static constexpr uint8_t  SCHEMA_VERSION = {s.name.upper()}_SCHEMA_VERSION;")
        lines.append(f"    static constexpr const char* TYPE_NAME = \"{s.name}\";")
        lines.append(f"}};")
        lines.append("extern \"C\" {")
        lines.append("#endif")
        lines.append("")

        # Alias for C++ compatibility — forward declare template
        # Actually we'll just put TYPE_ID, SCHEMA_VERSION, TYPE_NAME as static constexpr in struct wrapper

        # Serialize function
        lines.extend(self._gen_serialize(s, type_id))

        # Deserialize function
        lines.extend(self._gen_deserialize(s, type_id))

        # Endian swap function
        lines.extend(self._gen_endian_swap(s))

        # Field descriptor table (field-level schema)
        lines.extend(self._gen_field_table(s))

        # Register function
        lines.extend(self._gen_register_func(s, type_id))

        return lines

    def _gen_field_table(self, s: StructDef) -> list[str]:
        lines = []
        lines.append(f"/** Field-level schema descriptor table for {s.name}. */")
        lines.append(f"static const FieldDesc {s.name}_fields[] = {{")
        for f in s.fields:
            elem_size = self._field_size(f)
            arr = f.array_size if f.array_size > 0 else 1
            lines.append(
                f"    {{ \"{f.name}\", {self._field_kind(f)}, "
                f"(uint16_t)offsetof({s.name}, {f.name}), "
                f"{elem_size}, {arr} }},")
        lines.append(f"}};")
        lines.append(f"#define {s.name.upper()}_FIELD_COUNT  "
                     f"(sizeof({s.name}_fields)/sizeof({s.name}_fields[0]))")
        lines.append("")
        return lines

    def _gen_serialize(self, s: StructDef, type_id: int) -> list[str]:
        lines = []
        lines.append(f"/** Serialize {s.name} to buffer (endian-aware). */")
        lines.append(f"static inline int {s.name}_serialize(const {s.name}* src,")
        lines.append(f"        uint8_t* buf, size_t* out_size) {{")
        lines.append(f"    if (!src) return -1;")
        lines.append(f"    size_t total = {self._struct_size(s)};")
        lines.append(f"    if (out_size) *out_size = total;")
        lines.append(f"    if (!buf) return 0;  /* size query only */")
        lines.append(f"")

        offset = 0
        for f in s.fields:
            elem_size = self._field_size(f)
            count = f.array_size if f.array_size > 0 else 1
            for i in range(count):
                access = f"{f.name}[{i}]" if f.array_size > 0 else f.name
                if f.is_nested:
                    lines.append(f"    {{ /* {access} */")
                    lines.append(f"        size_t sz = 0;")
                    lines.append(f"        {f.idl_type}_serialize(&src->{access}, buf ? buf + {offset} : NULL, &sz);")
                    lines.append(f"    }}")
                elif f.is_enum:
                    lines.append(f"    if (buf) buf[{offset}] = (int8_t)src->{access};")
                elif is_float_type(f.idl_type) and f.idl_type == 'float64':
                    lines.append(f"    if (buf) memcpy(buf + {offset}, &src->{access}, 8);")
                    # TODO: endian swap for cross-platform
                elif f.idl_type == 'float':
                    lines.append(f"    if (buf) memcpy(buf + {offset}, &src->{access}, 4);")
                elif is_integer_type(f.idl_type) and f.idl_type in ('int16', 'uint16'):
                    lines.append(f"    if (buf) memcpy(buf + {offset}, &src->{access}, 2);")
                elif is_integer_type(f.idl_type) and f.idl_type in ('int32', 'uint32'):
                    lines.append(f"    if (buf) memcpy(buf + {offset}, &src->{access}, 4);")
                elif is_integer_type(f.idl_type) and f.idl_type in ('int64', 'uint64'):
                    lines.append(f"    if (buf) memcpy(buf + {offset}, &src->{access}, 8);")
                else:
                    lines.append(f"    if (buf) buf[{offset}] = (uint8_t)src->{access};")
                offset += elem_size

        lines.append(f"    return 0;")
        lines.append(f"}}")
        lines.append("")
        return lines

    def _gen_deserialize(self, s: StructDef, type_id: int) -> list[str]:
        lines = []
        lines.append(f"/** Deserialize {s.name} from buffer (endian-aware). */")
        lines.append(f"static inline int {s.name}_deserialize({s.name}* dst,")
        lines.append(f"        const uint8_t* buf, size_t size) {{")
        lines.append(f"    if (!dst || !buf) return -1;")
        lines.append(f"    if (size < {self._struct_size(s)}) return -1;")
        lines.append(f"    memset(dst, 0, sizeof(*dst));")
        lines.append(f"")

        offset = 0
        for f in s.fields:
            elem_size = self._field_size(f)
            count = f.array_size if f.array_size > 0 else 1
            for i in range(count):
                access = f"{f.name}[{i}]" if f.array_size > 0 else f.name
                if f.is_nested:
                    nested_size = self._struct_size(self.structs[f.nested_typename])
                    lines.append(f"    {{ /* {access} */")
                    lines.append(f"        {f.idl_type}_deserialize(&dst->{access}, buf + {offset}, {nested_size});")
                    lines.append(f"    }}")
                elif f.is_enum:
                    lines.append(f"    dst->{access} = ({f.c_type})((int8_t)buf[{offset}]);")
                elif is_float_type(f.idl_type) and f.idl_type == 'float64':
                    lines.append(f"    memcpy(&dst->{access}, buf + {offset}, 8);")
                elif f.idl_type == 'float':
                    lines.append(f"    memcpy(&dst->{access}, buf + {offset}, 4);")
                elif is_integer_type(f.idl_type) and f.idl_type in ('int16', 'uint16'):
                    lines.append(f"    memcpy(&dst->{access}, buf + {offset}, 2);")
                elif is_integer_type(f.idl_type) and f.idl_type in ('int32', 'uint32'):
                    lines.append(f"    memcpy(&dst->{access}, buf + {offset}, 4);")
                elif is_integer_type(f.idl_type) and f.idl_type in ('int64', 'uint64'):
                    lines.append(f"    memcpy(&dst->{access}, buf + {offset}, 8);")
                else:
                    lines.append(f"    dst->{access} = ({f.c_type})buf[{offset}];")
                offset += elem_size

        lines.append(f"    return 0;")
        lines.append(f"}}")
        lines.append("")
        return lines

    def _gen_endian_swap(self, s: StructDef) -> list[str]:
        lines = []
        lines.append(f"/** In-place endian swap for {s.name}. */")
        lines.append(f"static inline void {s.name}_endian_swap(void* data) {{")
        lines.append(f"    if (!data) return;")
        lines.append(f"    uint8_t* p = (uint8_t*)data;")
        lines.append("")

        offset = 0
        for f in s.fields:
            elem_size = self._field_size(f)
            count = f.array_size if f.array_size > 0 else 1
            for i in range(count):
                if f.is_nested:
                    lines.append(f"    {f.idl_type}_endian_swap(p + {offset});")
                elif elem_size == 2:
                    lines.append(f"    serializer_swap16(p + {offset});")
                elif elem_size == 4:
                    lines.append(f"    serializer_swap32(p + {offset});")
                elif elem_size == 8:
                    lines.append(f"    serializer_swap64(p + {offset});")
                # 1-byte types need no swap
                offset += elem_size

        lines.append("}")
        lines.append("")
        return lines

    def _gen_register_func(self, s: StructDef, type_id: int) -> list[str]:
        lines = []
        lines.append(f"/** Auto-register {s.name} in type registry. */")
        lines.append(f"static inline void {s.name}_register_type(void) {{")
        lines.append(f"    TypeRegistryEntry e = {{")
        lines.append(f"        .type_id        = {s.name.upper()}_TYPE_ID,")
        lines.append(f"        .schema_version = {s.name.upper()}_SCHEMA_VERSION,")
        lines.append(f"        .struct_size    = sizeof({s.name}),")
        lines.append(f"        .type_name      = \"{s.name}\",")
        lines.append(f"        .serialize      = (SerializeFunc){s.name}_serialize,")
        lines.append(f"        .deserialize    = (DeserializeFunc){s.name}_deserialize,")
        lines.append(f"        .endian_swap    = (EndianSwapFunc){s.name}_endian_swap,")
        lines.append(f"        .schema_hash    = {s.name.upper()}_SCHEMA_HASH,")
        lines.append(f"        .fields         = {s.name}_fields,")
        lines.append(f"        .field_count    = (uint16_t){s.name.upper()}_FIELD_COUNT,")
        lines.append(f"    }};")
        lines.append(f"    serializer_register_type(&e);")
        lines.append(f"}}")
        lines.append("")
        return lines

    def _gen_registration(self) -> list[str]:
        lines = []
        lines.append("/* ── Batch registration ─────────────────────────────── */")
        lines.append("")
        lines.append("/**")
        lines.append(" * Register all ADAS message types in the type registry.")
        lines.append(" * Call once during initialization (idempotent).")
        lines.append(" */")
        lines.append("static inline void adas_msgs_register_all(void) {")
        for name in self._struct_order:
            lines.append(f"    {name}_register_type();")
        lines.append("}")
        lines.append("")
        return lines


# =============================================================================
# Main
# =============================================================================

def main():
    import argparse

    ap = argparse.ArgumentParser(description="FlowEngine IDL Code Generator")
    ap.add_argument("input", help="Input .msg IDL file")
    ap.add_argument("output", nargs="?", default=None,
                    help="Output .h file (monolith mode, default: stdout)")
    ap.add_argument("--split", action="store_true",
                    help="Generate per-struct split headers under <output_dir>/gen/")
    ap.add_argument("--output-dir", default=None,
                    help="Root output directory for --split mode (default: dirname of output)")
    args = ap.parse_args()

    with open(args.input, 'r') as f:
        text = f.read()

    parser = IDLParser(text)
    parser.parse()

    gen = CodeGenerator(parser)

    if args.split:
        # Determine output directory: --output-dir > dirname(output) > .
        output_dir = args.output_dir
        if output_dir is None and args.output:
            output_dir = os.path.dirname(args.output) or "."
        elif output_dir is None:
            output_dir = "."

        msg_basename = os.path.splitext(os.path.basename(args.input))[0]
        generated = gen.generate_split(output_dir, msg_basename)

        for rel in generated:
            print(f"  Generated: {rel}")
        print(f"Split generation: {len(parser.structs)} structs, {len(parser.enums)} enums "
              f"→ {len(generated)} files")
    else:
        output = gen.generate()
        if args.output:
            os.makedirs(os.path.dirname(args.output), exist_ok=True)
            with open(args.output, 'w') as f:
                f.write(output)
            print(f"Generated: {args.output} ({len(parser.structs)} structs, {len(parser.enums)} enums)")
        else:
            print(output)


if __name__ == '__main__':
    main()
