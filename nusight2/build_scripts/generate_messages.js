const fs = require('fs')
const path = require('path')
const pbjs = require('protobufjs/cli/pbjs')
const pbts = require('protobufjs/cli/pbts')

const REPO_ROOT = path.resolve(__dirname, '..', '..')
const NUSIGHT_ROOT = path.resolve(__dirname, '..')

const messagesDir = path.resolve(REPO_ROOT, 'shared/message')
const nuclearMessagesDir = path.resolve(REPO_ROOT, 'nuclear/message/proto')
const googleMessagesDir = path.resolve(NUSIGHT_ROOT, 'src/shared/proto')

const messageSourceDirs = [messagesDir, nuclearMessagesDir, googleMessagesDir]

const messageSourceFiles = [
  `${messagesDir}/**/*.proto`,
  `${nuclearMessagesDir}/**/*.proto`,
  `${googleMessagesDir}/**/*.proto`,
]

function generateMessagesJs(outputFilePath) {
  // prettier-ignore
  const args = [
    '--target', 'static-module',
    '--wrap', 'es6',
    '--out', outputFilePath,
    '--no-create', '--no-verify', '--no-convert', '--no-delimited',
    ...messageSourceDirs.map(dir => ['--path', dir]).flat(),
    ...messageSourceFiles
  ]

  return new Promise((resolve, reject) => {
    pbjs.main(args, err => {
      if (err) {
        reject(err)
      } else {
        console.log(`Generated JS: ${outputFilePath}`)
        resolve()
      }
    })
  })
}

function generateMessagesTs(inputFilePath, outputFilePath) {
  // prettier-ignore
  const args = [
    inputFilePath,
    '--out', outputFilePath,
    '--name', '',
  ]

  return new Promise((resolve, reject) => {
    pbts.main(args, err => {
      if (err) {
        reject(err)
      } else {
        console.log(`Generated TS: ${outputFilePath}`)
        resolve()
      }
    })
  })
}

function generateMessageFields() {
  // prettier-ignore
  const args = [
    '--target', 'json',
    ...messageSourceDirs.map(dir => ['--path', dir]).flat(),
    ...messageSourceFiles
  ]

  return new Promise((resolve, reject) => {
    pbjs.main(args, function (err, output) {
      if (err) {
        reject(err)
      } else {
        resolve(JSON.parse(output))
      }
    })
  })
}

async function generateMessageFieldsIndex(outputFilePath) {
  const messageFieldsDesc = await generateMessageFields()

  const index = indexMessageFields(messageFieldsDesc, ['id'])

  const types = `
export interface MessageField {
  wireType: number
  type: string
  id: number
}

export interface MessageFieldsIndex {
  [messageName: string]: {
    [fieldName: string]: MessageField
  }
}
  `.trim()

  const file =
    types +
    '\n\n' +
    'export const messageFieldsIndex: MessageFieldsIndex = ' +
    JSON.stringify(index, null, '  ')

  await fs.promises.writeFile(outputFilePath, file)

  console.log(`Generated message fields index: ${outputFilePath}`)
}

function indexMessageFields(messagesDesc, fieldsToIndex) {
  const index = {}

  processIndex(messagesDesc, index, {
    parentKey: '',
    messagePath: '',
    fieldsToIndex,
  })

  return index
}

// Data types to protobuf wire types
const protobufWireTypes = {
  // Varint fields
  int32: 0,
  int64: 0,
  uint32: 0,
  uint64: 0,
  sint32: 0,
  sint64: 0,
  bool: 0,
  enum: 0,

  // 64-bit fields
  fixed64: 1,
  sfixed64: 1,
  double: 1,

  // Length-delimited fields
  string: 2,
  bytes: 2,

  // Note: wire types for composite fields (i.e. embedded messages, groups, packed repeated fields)
  // are omitted from this list

  // 32-bit fields
  fixed32: 5,
  sfixed32: 5,
  float: 5,
}

/**
 * The protobuf.js JSON description object has the following shape:
 *    {
 *      "nested": {
 *        "namespaceA": {
 *          "nested": {
 *            "namespaceB": {
 *              "nested": {
 *                "MessageTypeA": {
 *                  "fields": {
 *                    "fieldA": {
 *                      "type": "double",
 *                      "id": 1
 *                    },
 *                    "fieldB": {
 *                      "type": "uint32",
 *                      "id": 2
 *                    }
 *                  },
 *                  "nested": {
 *                    "SubMessageA": {...},
 *                    "SubMessageB": {...}
 *                  }
 *                },
 *                "MessageTypeB": {...}
 *              }
 *            }
 *          }
 *        }
 *      }
 *    }
 *
 * This function walks the description and flattens it into an index object where the keys are
 * fully-qualified message names, and the values are descriptions of the indexed fields.
 */
function processIndex(descNode, index, { parentKey, messagePath, fieldsToIndex }) {
  for (const [nodeKey, nodeValue] of Object.entries(descNode)) {
    // If the key is 'fields', we've arrived at the fields for a message
    if (nodeKey === 'fields') {
      for (const [fieldName, fieldDesc] of Object.entries(nodeValue)) {
        // Skip to the next field if this field is not in the list of fields to index
        if (!fieldsToIndex.includes(fieldName)) {
          continue
        }
        // Otherwise add the field to the message it belongs to in the index
        else {
          const indexEntry = index[messagePath] ?? {}

          indexEntry[fieldName] = fieldDesc
          indexEntry[fieldName].wireType = protobufWireTypes[fieldDesc.type] ?? -1

          index[messagePath] = indexEntry
        }
      }
    }

    // We've arrived at a node that has nested nodes. This could be a namespace,
    // or a message that has sub messages: process it recursively.
    else if (nodeKey === 'nested') {
      processIndex(nodeValue, index, { messagePath, parentKey: nodeKey, fieldsToIndex })
    }

    // The description node we're processing came from the key 'nested', so it will have
    // messages as its direct children. Process them here recursively, appending the
    // current node's key to the message path.
    else if (parentKey === 'nested') {
      processIndex(nodeValue, index, {
        messagePath: messagePath.length === 0 ? nodeKey : `${messagePath}.${nodeKey}`,
        parentKey: nodeKey,
        fieldsToIndex,
      })
    }
  }
}

async function main() {
  const messagesOutputDir = path.resolve(NUSIGHT_ROOT, 'src/shared')

  const messagesJsFile = path.join(messagesOutputDir, 'messages.js')
  const messagesTsFile = path.join(messagesOutputDir, 'messages.d.ts')
  const messagesIndexFile = path.join(messagesOutputDir, 'message_fields_index.ts')

  generateMessagesJs(messagesJsFile).then(() => {
    generateMessagesTs(messagesJsFile, messagesTsFile)
  })

  await generateMessageFieldsIndex(messagesIndexFile)
}

main()
