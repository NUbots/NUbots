import java.io.*;
import org.json.*;


class ConfigConverter
{
   static ConfigScanner confScan;

    public static void main(String args[])
    {
        if(args.length == 0)
        {
            System.out.println("Usage: $ java ConfigConverter input.cfg");
            return;
        }
        
        for(String currArg : args)
            try
            {
                FileInputStream fstream = new FileInputStream(currArg);
                // Get the object of DataInputStream
                DataInputStream in = new DataInputStream(fstream);
                LineNumberReader br = new LineNumberReader(new InputStreamReader(in));
                br.setLineNumber(1);

                confScan = new ConfigScanner(br);


                String rootName = currArg.split("\\.")[0];

                JSONObject jsonRoot = new JSONObject();
                //For each token in the file
                Token token;
                while ((token = confScan.peekNextToken(br)) != null)
                {
                    // System.out.println("\ntoken: " + token);
                    if(rootName.equals("Camera"))
                        readCameraConfig(jsonRoot, br); 
                    
                    if(rootName.startsWith("Replacement"))
                        readReplacementRulesConfig(jsonRoot, br); 

                    if(rootName.startsWith("WayPoints"))
                        readWayPointsConfig(jsonRoot, br); 
                }
                
                JSONObject jsonOut = new JSONObject();
                jsonOut.put(rootName, jsonRoot);
                System.out.println(jsonOut.toString(4));
                writeStringToFile(jsonOut.toString(4), rootName + ".json");

            }
            catch (FileNotFoundException e)
            {
                e.printStackTrace();
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
            catch(JSONException e)
            {
                e.printStackTrace();
            }
        }


    public static void readCameraConfig(JSONObject jsonRoot, LineNumberReader br) throws IOException, JSONException
    {
        Token nameTok = confScan.getCheckedToken(br, Token.WORD);
        confScan.getCheckedToken(br, Token.COLON);
        Token valTok = confScan.getCheckedToken(br, Token.NUMBER);
        
        JSONObject rangeObj = getJSONRange(br);

        // read rest of line as a comment
        br.skip(1);
        String commentStr = br.readLine();

        JSONObject paramObj = 
            makeParamObj
            (
                "float",
                valTok.lexeme,
                commentStr,
                rangeObj
            );
        
        jsonRoot.put(nameTok.lexeme, paramObj);
    }

    public static void readReplacementRulesConfig(JSONObject jsonRoot, LineNumberReader br) throws IOException, JSONException
    {
        JSONObject jsonObj = new JSONObject();

        Token nameTok = confScan.getCheckedToken(br, Token.WORD);
        confScan.checkTokens(br, Token.COLON);

        
        for(int f = 0; f < 3; f++)
        {
            // read 'before'/'after'/'middle'
            Token beforeTok = confScan.getCheckedToken(br, Token.WORD);
            JSONObject beforeObj = new JSONObject();
            confScan.checkTokens(br, Token.COLON);
            
            confScan.checkTokens(br, Token.LPAREN);
            Token minTok = confScan.getCheckedToken(br, Token.NUMBER);
            confScan.checkTokens(br, Token.COMMA);
            Token maxTok = confScan.getCheckedToken(br, Token.NUMBER);
            confScan.checkTokens(br, Token.RPAREN);

            beforeObj.put("min", minTok.lexeme);
            beforeObj.put("max", maxTok.lexeme);

            JSONArray classArray = getJSONArray(br, Token.WORD);
            beforeObj.put("classes", classArray);

            // read rest of line as a comment
            if(confScan.peekNextToken(br).type == Token.SLASH)
            {
                confScan.checkTokens(br, Token.SLASH, Token.SLASH);
                String commentStr = br.readLine();
                beforeObj.put("desc", commentStr);
            }
            else
                beforeObj.put("desc", "");

            jsonObj.put(beforeTok.lexeme, beforeObj);
        }

        Token replTok = confScan.getCheckedToken(br, Token.WORD);
        confScan.checkTokens(br, Token.COLON);
        Token replType = confScan.getCheckedToken(br, Token.WORD);
        jsonObj.put(replTok.lexeme, replType.lexeme);
        String commentStr = br.readLine();
        
        jsonRoot.put(nameTok.lexeme, jsonObj);
    }

    public static void readWayPointsConfig(JSONObject jsonRoot, LineNumberReader br) throws IOException, JSONException
    {
        Token nameTok = confScan.getCheckedToken(br, Token.WORD);
        confScan.checkTokens(br, Token.COLON);

        JSONArray arr = getJSONArrayND(
            br,
            Token.NUMBER,
            2,
            Token.LBRACKET,
            Token.RBRACKET,
            Token.COMMA
            );
        JSONObject arrObj = makeParamObj("vector_float", arr, "", null);

        jsonRoot.put(nameTok.lexeme, arrObj);
    }


    static JSONObject makeParamObj(
        String valType, 
        JSONArray valObj, 
        String desc, 
        JSONObject rangeObj
        ) throws IOException, JSONException 
    {
        JSONObject obj = new JSONObject();

        obj.put("value", valObj);
        obj.put("type", valType);
        obj.put("desc", desc);
        
        if(rangeObj != null) obj.put("range", rangeObj);
        else
        {
            // null range
            JSONObject ro = new JSONObject();
            ro.put("min", 0);
            ro.put("max", 0);
            ro.put("lBound", "NONE");
            ro.put("uBound", "NONE");
            ro.put("outside", "false");

            obj.put("range", ro);
        }

        return obj;
    }
    static JSONObject makeParamObj(
        String valType, 
        String valStr, 
        String desc, 
        JSONObject rangeObj
        ) throws IOException, JSONException
    {
        JSONObject obj = new JSONObject();

        obj.put("value", valStr);
        obj.put("type", valType);
        obj.put("desc", desc);
        
        if(rangeObj != null) obj.put("range", rangeObj);
        else
        {
            // null range
            JSONObject ro = new JSONObject();
            ro.put("min", 0);
            ro.put("max", 0);
            ro.put("lBound", "NONE");
            ro.put("uBound", "NONE");
            ro.put("outside", "false");

            obj.put("range", ro);
        }

        return obj;
    }


    static JSONArray getJSONArray(LineNumberReader br, int tokenType) throws IOException, JSONException
    {
        return getJSONArray(br, tokenType, Token.LBRACKET, Token.RBRACKET, Token.COMMA);
    }

    static JSONArray getJSONArray(
        LineNumberReader br, 
        int tokenType,
        int lDelimType,
        int rDelimType,
        int sepType   
        ) throws IOException, JSONException
    {
        JSONArray arrayObj = new JSONArray();

        confScan.checkTokens(br, lDelimType);
        for(;;)
        {
            Token listTok = confScan.peekNextToken(br);

            if(listTok.type == tokenType)
            {
                arrayObj.put(listTok.lexeme);
                confScan.skipTokens(br, 1);
            } else break;

            if(confScan.peekNextToken(br).type != sepType) break;
            confScan.skipTokens(br, 1);
        }
        confScan.checkTokens(br, rDelimType);

        return arrayObj;
    }

    static JSONArray getJSONArrayND(
        LineNumberReader br, 
        int tokenType,
        int numDimensions,
        int lDelimType,
        int rDelimType,
        int sepType
        ) throws IOException, JSONException
    {
        JSONArray arrayObj = new JSONArray();

        // opening bracket
        confScan.checkTokens(br, lDelimType);

        Token peek = confScan.peekNextToken(br);

        if(peek.type == lDelimType)
        {
            arrayObj.put(getJSONArrayND(br, tokenType, numDimensions, lDelimType, rDelimType, sepType));
            for(;;)
            {
                if(confScan.peekNextToken(br).type == sepType) confScan.getCheckedToken(br, sepType);
                if(confScan.peekNextToken(br).type != lDelimType) break;
                arrayObj.put(getJSONArrayND(br, tokenType, numDimensions, lDelimType, rDelimType, sepType));
            }
        }
        else
        {
            Token lItem = confScan.getCheckedToken(br, tokenType);
            arrayObj.put(lItem.lexeme);
            for(;;)
            {
                if(confScan.peekNextToken(br).type != sepType) break;
                confScan.getCheckedToken(br, sepType);

                lItem = confScan.getCheckedToken(br, tokenType);
                arrayObj.put(lItem.lexeme);
            }
        }

        // closing bracket
        confScan.checkTokens(br, rDelimType);
        return arrayObj;
    }

    static JSONObject getJSONRange(LineNumberReader br) throws IOException, JSONException
    {
        return getJSONRange(br, Token.LBRACKET, Token.RBRACKET);
    }
    static JSONObject getJSONRange(LineNumberReader br, int lDelimType, int rDelimType) throws IOException, JSONException
    {
        JSONArray rArr = 
            getJSONArray
            (
                br, 
                Token.NUMBER, 
                lDelimType, 
                rDelimType, 
                Token.COMMA
            );

        JSONObject ro = new JSONObject();
        ro.put("min", rArr.get(0));
        ro.put("max", rArr.get(1));
        ro.put("lBound", "CLOSED");
        ro.put("uBound", "CLOSED");
        ro.put("outside", "false");

        return ro;
    }


    static void writeStringToFile(String str, String fName)
    {
        try
        {
          FileWriter fstream = new FileWriter(fName);
          BufferedWriter out = new BufferedWriter(fstream);
          out.write(str);
          out.close();
        }
        catch (Exception e)
        {
          System.err.println("Error: " + e.getMessage());
        }
    }



}
