// PostToolUse(Write) hook: prepend the STM32CubeIDE-style header comment
// to newly written .c/.h files that don't have one yet.
//   .c -> header comment only
//   .h -> header comment + #ifndef/#define/#endif include guard
//
// Reads the hook event JSON from stdin (tool_name, tool_input.file_path).
// Never blocks the tool: any error is reported on stderr only.
'use strict';

const fs = require('fs');

let raw = '';
process.stdin.setEncoding('utf8');
process.stdin.on('data', (chunk) => { raw += chunk; });
process.stdin.on('end', () => {
  try {
    const evt = JSON.parse(raw);
    if (evt.tool_name !== 'Write') return;

    const p = evt.tool_input && evt.tool_input.file_path;
    if (typeof p !== 'string') return;
    if (!/\.(c|h)$/i.test(p)) return;
    if (!fs.existsSync(p)) return;

    // Read raw bytes so the original encoding is preserved untouched.
    const body = fs.readFileSync(p);
    if (body.includes(Buffer.from('Created on:'))) return; // header already present

    const name = p.replace(/\\/g, '/').split('/').pop();
    const now = new Date();
    const date = now.getFullYear() + '年' + (now.getMonth() + 1) + '月' + now.getDate() + '日';
    const eol = body.includes(Buffer.from('\r\n')) ? '\r\n' : '\n';

    let text = '/*' + eol +
      ' * ' + name + eol +
      ' *' + eol +
      ' *  Created on: ' + date + eol +
      ' *      Author: twyyd' + eol +
      ' */' + eol;

    if (/\.h$/i.test(p)) {
      const base = name.replace(/\.h$/i, '').toUpperCase().replace(/[^A-Z0-9_]/g, '_');
      const guard = 'INC_' + base + '_H_';
      text += eol + eol +
        '#ifndef ' + guard + eol +
        '#define ' + guard + eol +
        eol + eol +
        '#endif /* ' + guard + ' */' + eol;
    }

    fs.writeFileSync(p, Buffer.concat([Buffer.from(text, 'utf8'), body]));
  } catch (err) {
    console.error('add-c-header hook error: ' + err.message);
  }
});
