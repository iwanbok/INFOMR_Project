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
/*     */ public class CleanOff
/*     */ {
/*     */   boolean outputNormals;
/*     */   int numNewVertices;
/*  29 */   int numVertices = 0; int numEdges;
/*  30 */   int numFaces = 0;
/*  31 */   double[][] vertices = null; int[] realIndex;
/*  32 */   int[] faceSizes = null;
/*  33 */   int[][] faces = null;
/*  34 */   double epsilon = 1.401298464324817E-45D;
/*     */ 
/*     */ 
/*     */   
/*     */   public static void main(String[] paramArrayOfString) {
/*  39 */     CleanOff cleanOff = new CleanOff();
/*     */     
/*  41 */     if (paramArrayOfString == null || paramArrayOfString.length < 2) {
/*  42 */       System.out.println("Syntax: CleanOff <source-file> <output-file> [<epsilon>]");
/*  43 */       System.exit(0);
/*     */     } 
/*     */     
/*     */     try {
/*  47 */       cleanOff.read(new BufferedReader(new FileReader(paramArrayOfString[0])));
/*  48 */       if (paramArrayOfString.length == 3) {
/*  49 */         double d = cleanOff.toDouble(paramArrayOfString[2]);
/*  50 */         cleanOff.setEpsilon(d);
/*     */       } 
/*  52 */       cleanOff.clean();
/*  53 */       cleanOff.write(new PrintStream(new FileOutputStream(paramArrayOfString[1])));
/*     */     } catch (Exception exception) {
/*  55 */       exception.printStackTrace();
/*  56 */       System.exit(1);
/*     */     } 
/*     */   }
/*     */ 
/*     */ 
/*     */ 
/*     */   
/*     */   public void read(BufferedReader paramBufferedReader) throws IOException {
/*  64 */     byte b1 = 3;
/*  65 */     this.numEdges = 0;
/*     */     
/*  67 */     String str = paramBufferedReader.readLine();
/*  68 */     if (str.equals("OFF")) {
/*  69 */       this.outputNormals = false;
/*  70 */     } else if (str.equals("NOFF")) {
/*  71 */       this.outputNormals = true;
/*  72 */       b1 = 6;
/*     */     } else {
/*  74 */       throw new IOException("Not supported input file type");
/*     */     } 
/*     */     
/*  77 */     StringTokenizer stringTokenizer = new StringTokenizer(paramBufferedReader.readLine());
/*  78 */     this.numVertices = toInt(stringTokenizer.nextToken());
/*  79 */     this.numFaces = toInt(stringTokenizer.nextToken());
/*  80 */     this.vertices = new double[this.numVertices][];
/*  81 */     this.faceSizes = new int[this.numFaces];
/*  82 */     this.faces = new int[this.numFaces][];
/*     */     
/*  84 */     for (byte b2 = 0; b2 < this.numVertices; b2++) {
/*  85 */       stringTokenizer = new StringTokenizer(paramBufferedReader.readLine());
/*  86 */       this.vertices[b2] = new double[b1];
/*  87 */       for (byte b = 0; b < b1; b++) {
/*  88 */         this.vertices[b2][b] = toDouble(stringTokenizer.nextToken());
/*     */       }
/*     */     } 
/*     */     
/*  92 */     for (byte b3 = 0; b3 < this.numFaces; b3++) {
/*  93 */       stringTokenizer = new StringTokenizer(paramBufferedReader.readLine());
/*  94 */       this.faceSizes[b3] = toInt(stringTokenizer.nextToken());
/*  95 */       this.numEdges += this.faceSizes[b3];
/*  96 */       this.faces[b3] = new int[this.faceSizes[b3]];
/*  97 */       for (byte b = 0; b < this.faceSizes[b3]; b++) {
/*  98 */         this.faces[b3][b] = toInt(stringTokenizer.nextToken());
/*     */       }
/*     */     } 
/*     */   }
/*     */ 
/*     */   
/* 104 */   public void setEpsilon(double paramDouble) { this.epsilon = paramDouble; }
/*     */ 
/*     */   
/*     */   public void clean() {
/* 108 */     this.realIndex = new int[this.numVertices];
/* 109 */     for (byte b1 = 0; b1 < this.numVertices; b1++) {
/* 110 */       this.realIndex[b1] = -1;
/*     */     }
/*     */     
/* 113 */     this.numNewVertices = 0;
/*     */     
/* 115 */     for (byte b2 = 0; b2 < this.numVertices; b2++) {
/*     */       
/* 117 */       if (this.realIndex[b2] == -1) {
/*     */         
/* 119 */         for (byte b = b2 + true; b < this.numVertices; b++) {
/*     */           
/* 121 */           if (this.realIndex[b] == -1)
/*     */           {
/* 123 */             if (isSameVertex(this.vertices[b2], this.vertices[b]))
/*     */             {
/* 125 */               this.realIndex[b] = this.numNewVertices;
/*     */             }
/*     */           }
/*     */         } 
/* 129 */         this.realIndex[b2] = this.numNewVertices;
/* 130 */         this.numNewVertices++;
/*     */       } 
/*     */     } 
/*     */ 
/*     */     
/* 135 */     int i = 0;
/* 136 */     for (byte b3 = 0; b3 < this.numFaces; b3++) {
/* 137 */       i = this.faceSizes[b3];
/* 138 */       for (byte b = 0; b < i; b++) {
/* 139 */         if (this.realIndex[this.faces[b3][b]] != -1) {
/* 140 */           this.faces[b3][b] = this.realIndex[this.faces[b3][b]];
/*     */         }
/*     */       } 
/*     */     } 
/*     */   }
/*     */   
/*     */   private boolean isSameVertex(double[] paramArrayOfDouble1, double[] paramArrayOfDouble2) {
/* 147 */     boolean bool = true;
/* 148 */     int i = paramArrayOfDouble1.length;
/* 149 */     byte b = 0;
/* 150 */     while (bool && b < i) {
/* 151 */       if (Math.abs(paramArrayOfDouble1[b] - paramArrayOfDouble2[b]) > this.epsilon) {
/* 152 */         bool = false;
/*     */       }
/* 154 */       b++;
/*     */     } 
/* 156 */     return bool;
/*     */   }
/*     */ 
/*     */   
/*     */   public void write(PrintStream paramPrintStream) {
/* 161 */     byte b1 = 3;
/*     */ 
/*     */     
/* 164 */     if (this.outputNormals) {
/* 165 */       paramPrintStream.println("NOFF");
/* 166 */       paramPrintStream.println(this.numNewVertices + " " + this.numFaces + " " + this.numEdges);
/* 167 */       b1 = 6;
/*     */     } else {
/* 169 */       paramPrintStream.println("OFF");
/* 170 */       paramPrintStream.println(this.numNewVertices + " " + this.numFaces + " " + this.numEdges);
/*     */     } 
/*     */     
/* 173 */     for (byte b2 = 0; b2 < this.numNewVertices; b2++) {
/* 174 */       byte b = 0;
/* 175 */       boolean bool = false;
/* 176 */       while (!bool && b < this.numVertices) {
/* 177 */         if (this.realIndex[b] == b2) {
/* 178 */           paramPrintStream.print(this.vertices[b][0] + "");
/* 179 */           for (byte b4 = 1; b4 < b1; b4++) {
/* 180 */             paramPrintStream.print(" " + this.vertices[b][b4]);
/*     */           }
/* 182 */           paramPrintStream.println();
/* 183 */           bool = true;
/*     */         } 
/* 185 */         b++;
/*     */       } 
/*     */     } 
/*     */     
/* 189 */     int i = 0;
/* 190 */     for (byte b3 = 0; b3 < this.numFaces; b3++) {
/* 191 */       i = this.faceSizes[b3];
/* 192 */       paramPrintStream.print(i + "");
/* 193 */       for (byte b = 0; b < i; b++) {
/* 194 */         paramPrintStream.print(" " + this.faces[b3][b]);
/*     */       }
/* 196 */       paramPrintStream.println();
/*     */     } 
/*     */   }
/*     */   
/*     */   private int toInt(String paramString) {
/* 201 */     int i = 0;
/*     */     
/*     */     try {
/* 204 */       i = Integer.parseInt(paramString);
/*     */     } catch (NumberFormatException numberFormatException) {
/* 206 */       System.err.println("Error: Expecting integer, found " + paramString);
/* 207 */       System.exit(1);
/*     */     } 
/* 209 */     return i;
/*     */   }
/*     */   
/*     */   private float toFloat(String paramString) {
/* 213 */     float f = 0.0F;
/*     */     
/*     */     try {
/* 216 */       f = Float.parseFloat(paramString);
/*     */     } catch (NumberFormatException numberFormatException) {
/* 218 */       System.err.println("Error: Expecting float, found " + paramString);
/* 219 */       System.exit(1);
/*     */     } 
/* 221 */     return f;
/*     */   }
/*     */   
/*     */   private double toDouble(String paramString) {
/* 225 */     double d = 0.0D;
/*     */     
/*     */     try {
/* 228 */       d = Double.parseDouble(paramString);
/*     */     } catch (NumberFormatException numberFormatException) {
/* 230 */       System.err.println("Error: Expecting double, found " + paramString);
/* 231 */       System.exit(1);
/*     */     } 
/* 233 */     return d;
/*     */   }
/*     */ }


/* Location:              C:\Users\TES Installatie\Downloads\cleanoff(1).jar!\CleanOff.class
 * Java compiler version: 1 (45.3)
 * JD-Core Version:       1.0.7
 */