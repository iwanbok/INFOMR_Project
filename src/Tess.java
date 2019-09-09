/*     */ import Geometry.Vector3D;
/*     */ import java.io.BufferedReader;
/*     */ import java.io.FileOutputStream;
/*     */ import java.io.FileReader;
/*     */ import java.io.IOException;
/*     */ import java.io.PrintStream;
/*     */ import java.util.StringTokenizer;
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ public class Tess
/*     */ {
/*     */   boolean hasTex = false;
/*     */   boolean hasNormals = false;
/*     */   boolean hasColor = false;
/*  37 */   int numVertices = 0;
/*  38 */   int numFaces = 0;
/*  39 */   int numEdges = 0;
/*  40 */   double[][] vertices = null; int numTris;
/*  41 */   int[] faceSizes = null;
/*  42 */   int[][] faces = null; int curTri;
/*  43 */   int[][] triangles = null;
/*     */ 
/*     */ 
/*     */   
/*     */   public static void main(String[] paramArrayOfString) {
/*  48 */     Tess tess = new Tess();
/*     */     
/*  50 */     if (paramArrayOfString == null || paramArrayOfString.length < 2) {
/*  51 */       System.out.println("Syntax: Tess <input-file> <output-file>");
/*  52 */       System.exit(0);
/*     */     } 
/*     */     
/*     */     try {
/*  56 */       tess.readOff(new BufferedReader(new FileReader(paramArrayOfString[0])));
/*  57 */       tess.tessellate();
/*  58 */       tess.write(new PrintStream(new FileOutputStream(paramArrayOfString[1])));
/*     */     } catch (Exception exception) {
/*  60 */       exception.printStackTrace();
/*  61 */       System.exit(1);
/*     */     } 
/*     */   }
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */   
/*     */   public void readOff(BufferedReader paramBufferedReader) throws IOException {
/*  70 */     byte b1 = 3;
/*     */     
/*  72 */     String str = paramBufferedReader.readLine();
/*  73 */     if (str.endsWith("OFF")) {
/*  74 */       if (str.startsWith("ST")) {
/*  75 */         this.hasTex = true;
/*  76 */         b1 += 2;
/*  77 */         str = str.substring(2);
/*     */       } 
/*  79 */       if (str.startsWith("C")) {
/*  80 */         this.hasColor = true;
/*  81 */         b1 += 3;
/*  82 */         str = str.substring(1);
/*     */       } 
/*  84 */       if (str.startsWith("N")) {
/*  85 */         this.hasNormals = true;
/*  86 */         b1 += 3;
/*  87 */         str = str.substring(1);
/*     */       } 
/*  89 */       if (str.length() != 3) {
/*  90 */         throw new IOException("Unsupported input file type given for OFF file.");
/*     */       }
/*     */     } else {
/*  93 */       throw new IOException("Unsupported input file type given for OFF file.");
/*     */     } 
/*     */     
/*  96 */     StringTokenizer stringTokenizer = new StringTokenizer(paramBufferedReader.readLine());
/*  97 */     this.numVertices = toInt(stringTokenizer.nextToken());
/*  98 */     this.numFaces = toInt(stringTokenizer.nextToken());
/*  99 */     this.vertices = new double[this.numVertices][];
/* 100 */     this.faceSizes = new int[this.numFaces];
/* 101 */     this.faces = new int[this.numFaces][];
/*     */     
/* 103 */     this.numTris = this.numFaces;
/*     */     
/* 105 */     for (byte b2 = 0; b2 < this.numVertices; b2++) {
/* 106 */       stringTokenizer = new StringTokenizer(paramBufferedReader.readLine());
/* 107 */       this.vertices[b2] = new double[b1];
/* 108 */       for (byte b = 0; b < b1; b++) {
/* 109 */         this.vertices[b2][b] = toDouble(stringTokenizer.nextToken());
/*     */       }
/*     */     } 
/*     */     
/* 113 */     for (byte b3 = 0; b3 < this.numFaces; b3++) {
/* 114 */       stringTokenizer = new StringTokenizer(paramBufferedReader.readLine());
/* 115 */       this.faceSizes[b3] = toInt(stringTokenizer.nextToken());
/* 116 */       this.numEdges += this.faceSizes[b3];
/* 117 */       if (this.faceSizes[b3] > 3) {
/* 118 */         this.numTris += this.faceSizes[b3] - 3;
/*     */       }
/* 120 */       this.faces[b3] = new int[this.faceSizes[b3]];
/* 121 */       for (byte b = 0; b < this.faceSizes[b3]; b++) {
/* 122 */         this.faces[b3][b] = toInt(stringTokenizer.nextToken());
/*     */       }
/*     */     } 
/*     */   }
/*     */ 
/*     */   
/*     */   public void tessellate() {
/* 129 */     this.curTri = 0;
/* 130 */     this.triangles = new int[this.numTris][3];
/*     */     
/* 132 */     for (byte b = 0; b < this.numFaces; b++) {
/* 133 */       if (this.faceSizes[b] == 3) {
/* 134 */         makeTriangle(this.faces[b][0], this.faces[b][1], this.faces[b][2]);
/*     */       } else {
/* 136 */         Vector3D vector3D = findNormal(b);
/* 137 */         if (convexFace(b, vector3D)) {
/* 138 */           int i = this.faceSizes[b] - 1;
/* 139 */           for (byte b1 = 1; b1 < i; b1++) {
/* 140 */             makeTriangle(this.faces[b][0], this.faces[b][b1], this.faces[b][b1 + true]);
/*     */           }
/*     */         }
/* 143 */         else if (this.faceSizes[b] == 4) {
/* 144 */           if (!leftTurn(vector3D, new Vector3D(this.vertices[this.faces[b][0]]), new Vector3D(this.vertices[this.faces[b][1]]), new Vector3D(this.vertices[this.faces[b][2]])) || !leftTurn(vector3D, new Vector3D(this.vertices[this.faces[b][2]]), new Vector3D(this.vertices[this.faces[b][3]]), new Vector3D(this.vertices[this.faces[b][0]]))) {
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */ 
/*     */             
/* 151 */             makeTriangle(this.faces[b][0], this.faces[b][1], this.faces[b][3]);
/* 152 */             makeTriangle(this.faces[b][1], this.faces[b][2], this.faces[b][3]);
/*     */           } else {
/* 154 */             makeTriangle(this.faces[b][0], this.faces[b][1], this.faces[b][2]);
/* 155 */             makeTriangle(this.faces[b][2], this.faces[b][3], this.faces[b][0]);
/*     */           } 
/*     */         } else {
/* 158 */           System.out.println("Error: Face " + b + " is not convex.");
/* 159 */           System.exit(1);
/*     */         } 
/*     */       } 
/*     */     } 
/*     */   }
/*     */ 
/*     */ 
/*     */   
/*     */   public void write(PrintStream paramPrintStream) {
/* 168 */     byte b1 = 3;
/*     */ 
/*     */     
/* 171 */     if (this.hasTex) {
/* 172 */       paramPrintStream.print("ST");
/* 173 */       b1 += 2;
/*     */     } 
/* 175 */     if (this.hasColor) {
/* 176 */       paramPrintStream.print("C");
/* 177 */       b1 += 3;
/*     */     } 
/* 179 */     if (this.hasNormals) {
/* 180 */       paramPrintStream.print("N");
/* 181 */       b1 += 3;
/*     */     } 
/*     */     
/* 184 */     paramPrintStream.println("OFF");
/* 185 */     paramPrintStream.println(this.numVertices + " " + this.numTris + " " + (this.numTris * 3));
/*     */ 
/*     */ 
/*     */     
/* 189 */     for (byte b2 = 0; b2 < this.numVertices; b2++) {
/* 190 */       paramPrintStream.print(this.vertices[b2][0] + "");
/* 191 */       for (byte b = 1; b < b1; b++) {
/* 192 */         paramPrintStream.print(" " + this.vertices[b2][b]);
/*     */       }
/* 194 */       paramPrintStream.println();
/*     */     } 
/*     */     
/* 197 */     for (byte b3 = 0; b3 < this.numTris; b3++) {
/* 198 */       paramPrintStream.println("3 " + this.triangles[b3][0] + " " + this.triangles[b3][1] + " " + this.triangles[b3][2]);
/*     */     }
/*     */   }
/*     */   
/*     */   private int toInt(String paramString) {
/* 203 */     int i = 0;
/*     */     
/*     */     try {
/* 206 */       i = Integer.parseInt(paramString);
/*     */     } catch (NumberFormatException numberFormatException) {
/* 208 */       System.err.println("Error. Expecting integer, found " + paramString);
/* 209 */       System.exit(1);
/*     */     } 
/* 211 */     return i;
/*     */   }
/*     */   
/*     */   private float toFloat(String paramString) {
/* 215 */     float f = 0.0F;
/*     */     
/*     */     try {
/* 218 */       f = Float.parseFloat(paramString);
/*     */     } catch (NumberFormatException numberFormatException) {
/* 220 */       System.err.println("Error. Expecting float, found " + paramString);
/* 221 */       System.exit(1);
/*     */     } 
/* 223 */     return f;
/*     */   }
/*     */   
/*     */   private double toDouble(String paramString) {
/* 227 */     double d = 0.0D;
/*     */     
/*     */     try {
/* 230 */       d = Double.parseDouble(paramString);
/*     */     } catch (NumberFormatException numberFormatException) {
/* 232 */       System.err.println("Error. Expecting double, found " + paramString);
/* 233 */       System.exit(1);
/*     */     } 
/* 235 */     return d;
/*     */   }
/*     */   
/*     */   private void makeTriangle(int paramInt1, int paramInt2, int paramInt3) {
/* 239 */     this.triangles[this.curTri] = new int[3];
/* 240 */     this.triangles[this.curTri][0] = paramInt1;
/* 241 */     this.triangles[this.curTri][1] = paramInt2;
/* 242 */     this.triangles[this.curTri][2] = paramInt3;
/* 243 */     this.curTri++;
/*     */   }
/*     */ 
/*     */   
/*     */   private Vector3D findNormal(int paramInt) {
/* 248 */     Vector3D vector3D1 = new Vector3D();
/* 249 */     Vector3D vector3D2 = new Vector3D();
/* 250 */     Vector3D vector3D3 = new Vector3D();
/*     */     
/* 252 */     int i = this.faceSizes[paramInt];
/* 253 */     vector3D2.loadFrom(this.vertices[this.faces[paramInt][i - 1]]);
/* 254 */     vector3D3.loadFrom(this.vertices[this.faces[paramInt][0]]);
/* 255 */     vector3D1.incBy(vector3D2.cross(vector3D3));
/*     */     
/* 257 */     for (byte b = 0; b < i - 1; b++) {
/* 258 */       vector3D2.loadFrom(this.vertices[this.faces[paramInt][b]]);
/* 259 */       vector3D3.loadFrom(this.vertices[this.faces[paramInt][b + true]]);
/* 260 */       vector3D1.incBy(vector3D2.cross(vector3D3));
/*     */     } 
/* 262 */     vector3D1.normalize();
/*     */     
/* 264 */     return vector3D1;
/*     */   }
/*     */ 
/*     */   
/*     */   private boolean convexFace(int paramInt, Vector3D paramVector3D) {
/* 269 */     boolean bool = true;
/*     */     
/* 271 */     Vector3D vector3D1 = new Vector3D();
/* 272 */     Vector3D vector3D2 = new Vector3D();
/* 273 */     Vector3D vector3D3 = new Vector3D();
/* 274 */     Vector3D vector3D4 = new Vector3D();
/*     */     
/* 276 */     byte b = 0;
/*     */     
/* 278 */     int i = this.faceSizes[paramInt];
/*     */     
/* 280 */     while (bool && b < i) {
/*     */       
/* 282 */       vector3D2.loadFrom(this.vertices[this.faces[paramInt][b]]);
/* 283 */       vector3D4.loadFrom(this.vertices[this.faces[paramInt][(b + true) % i]]);
/* 284 */       vector3D3.loadFrom(this.vertices[this.faces[paramInt][(b + 2) % i]]);
/*     */       
/* 286 */       vector3D3.decBy(vector3D4);
/* 287 */       vector3D2.decBy(vector3D4);
/*     */       
/* 289 */       vector3D1 = vector3D3.cross(vector3D2);
/*     */       
/* 291 */       if (vector3D1.dot(paramVector3D) < 0.0D) {
/* 292 */         bool = false;
/*     */       }
/* 294 */       b++;
/*     */     } 
/*     */     
/* 297 */     return bool;
/*     */   }
/*     */   
/*     */   private boolean leftTurn(Vector3D paramVector3D1, Vector3D paramVector3D2, Vector3D paramVector3D3, Vector3D paramVector3D4) {
/* 301 */     boolean bool = true;
/*     */     
/* 303 */     paramVector3D4.decBy(paramVector3D3);
/* 304 */     paramVector3D2.decBy(paramVector3D3);
/*     */     
/* 306 */     Vector3D vector3D = paramVector3D4.cross(paramVector3D2);
/*     */     
/* 308 */     if (vector3D.dot(paramVector3D1) < 0.0D) {
/* 309 */       bool = false;
/*     */     }
/*     */     
/* 312 */     return bool;
/*     */   }
/*     */ }


/* Location:              C:\Users\TES Installatie\Downloads\tess.jar!\Tess.class
 * Java compiler version: 1 (45.3)
 * JD-Core Version:       1.0.7
 */