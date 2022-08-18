//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// Author:  David Burken
//
// Description:  NITF reader for j2k images using OpenJPEG library.
//
// $Id$
//----------------------------------------------------------------------------


#include <openjpeg.h>

#include <ossimOpjNitfReader.h>
#include <ossimOpjCommon.h>
#include <ossim/base/ossimCommon.h>
#include <ossim/base/ossimException.h>
#include <ossim/base/ossimTrace.h>
//#include <ossim/base/ossimEndian.h>
//#include <ossim/imaging/ossimTiffTileSource.h>
//#include <ossim/imaging/ossimImageDataFactory.h>
#include <ossim/support_data/ossimNitfImageHeader.h>
#include <ossim/support_data/ossimJ2kCodRecord.h>
#include <ossim/support_data/ossimJ2kSizRecord.h>
#include <ossim/support_data/ossimJ2kSotRecord.h>
#include <ossim/support_data/ossimJ2kTlmRecord.h>
using namespace std;

#ifdef OSSIM_ID_ENABLED
static const char OSSIM_ID[] = "$Id";
#endif

static ossimTrace traceDebug("ossimOpjNitfReader:debug");


static const ossim_uint16 SOC_MARKER = 0xff4f; // start of codestream
static const ossim_uint16 SIZ_MARKER = 0xff51; // start of tile-part
static const ossim_uint16 SOT_MARKER = 0xff90; // start of tile-part

RTTI_DEF1_INST(ossimOpjNitfReader,
               "ossimOpjNitfReader",
               ossimNitfTileSource)



ossimOpjNitfReader::ossimOpjNitfReader()
   : ossimNitfTileSource()
{
}

ossimOpjNitfReader::~ossimOpjNitfReader()
{
   close();
}

bool ossimOpjNitfReader::canUncompress(
   const ossimNitfImageHeader* hdr) const
{
   if (!hdr)
   {
      return false;
   }
   if (hdr->getCompressionCode() == "C8") // jpeg
   {
      return true;
   }
   return false;
}

void ossimOpjNitfReader::initializeReadMode()
{
   // Initialize the read mode.
   theReadMode = READ_MODE_UNKNOWN;
   
   const ossimNitfImageHeader* hdr = getCurrentImageHeader();
   if (!hdr)
   {
      return;
   }

   if ( (hdr->getIMode() == "B") && (hdr->getCompressionCode()== "C8") )
   {
      theReadMode = READ_JPEG_BLOCK; 
   }
}

void ossimOpjNitfReader::initializeCompressedBuf()
{
   //---
   // If all block sizes are the same initialize theCompressedBuf; else,
   // we will allocate on each loadBlock.
   //---
   if (theNitfBlockSize.size() == 0)
   {
      theCompressedBuf.clear();
      return;
   }
   std::vector<ossim_uint32>::const_iterator i = theNitfBlockSize.begin();
   ossim_uint32 size = (*i);
   ++i;
   while (i != theNitfBlockSize.end())
   {
      if ((*i) != size)
      {
         theCompressedBuf.clear();
         return; // block sizes different
      }
      ++i;
   }
   theCompressedBuf.resize(size); // block sizes all the same.
}

bool ossimOpjNitfReader::scanForJpegBlockOffsets()
{
   const ossimNitfImageHeader* hdr = getCurrentImageHeader();
   
   if ( !hdr || (theReadMode != READ_JPEG_BLOCK) || !theFileStr )
   {
      return false;
   }

   theNitfBlockOffset.clear();
   theNitfBlockSize.clear();

   //---
   // NOTE:
   // SOC = 0xff4f Start of Codestream
   // SOT = 0xff90 Start of tile
   // SOD = 0xff93 Last marker in each tile
   // EOC = 0xffd9 End of Codestream
   //---
   char c;

   // Seek to the first block.
   theFileStr->seekg(hdr->getDataLocation(), ios::beg);
   if (theFileStr->fail())
   {
      return false;
   }
   
   // Read the first two bytes and verify it is SOC; if not, get out.
   theFileStr->get( c );
   if (static_cast<ossim_uint8>(c) != 0xff)
   {
      return false;
   }
   theFileStr->get(c);
   if (static_cast<ossim_uint8>(c) != 0x4f)
   {
      return false;
   }

   ossim_uint32 blockSize = 2;  // Read two bytes...

   // Add the first offset.
   // theNitfBlockOffset.push_back(hdr->getDataLocation());

   // Find all the SOC markers.
   while ( theFileStr->get(c) ) 
   {
      ++blockSize;
      if (static_cast<ossim_uint8>(c) == 0xff)
      {
         if ( theFileStr->get(c) )
         {
            ++blockSize;

            if (static_cast<ossim_uint8>(c) == 0x90) // At SOC marker...
            {
               std::streamoff pos = theFileStr->tellg();
               theNitfBlockOffset.push_back(pos-2);
            }
            else if (static_cast<ossim_uint8>(c) == 0x93) // At EOC marker...
            {
               // Capture the size of this block.
               theNitfBlockSize.push_back(blockSize);
               blockSize = 0;
            }
         }
      }
   }

   theFileStr->seekg(0, ios::beg);
   theFileStr->clear();

   // We should have the same amount of offsets as we do blocks...
   ossim_uint32 total_blocks =
      hdr->getNumberOfBlocksPerRow()*hdr->getNumberOfBlocksPerCol();
   
   if (theNitfBlockOffset.size() != total_blocks)
   {
      if (traceDebug())
      {
         ossimNotify(ossimNotifyLevel_WARN)
            << "DEBUG:"
            << "\nBlock offset count wrong!"
            << "\nblocks:  " << total_blocks
            << "\noffsets:  " << theNitfBlockOffset.size()
            << std::endl;
      }
      
      return false;
   }
   if (theNitfBlockSize.size() != total_blocks)
   {
      if (traceDebug())
      {
         ossimNotify(ossimNotifyLevel_WARN)
            << "DEBUG:"
            << "\nBlock size count wrong!"
            << "\nblocks:  " << total_blocks
            << "\nblock size array:  " << theNitfBlockSize.size()
            << std::endl;
      }

      return false;
   }

   return true;
}

std::ostream& ossimOpjNitfReader::dumpTiles(std::ostream& out)
{
   //---
   // NOTE:
   // SOC = 0xff4f Start of Codestream
   // SOT = 0xff90 Start of tile
   // SOD = 0xff93 Last marker in each tile
   // EOC = 0xffd9 End of Codestream
   //---
   
   const ossimNitfImageHeader* hdr = getCurrentImageHeader();
   if(hdr)
   {
      // Capture the starting position.
      std::streampos currentPos = theFileStr->tellg();
      
      // Seek to the first block.
      // theFileStr->seekg(m_startOfCodestreamOffset, ios_base::beg);
      if (theFileStr->good())
      {
         out << "offset to codestream: " << currentPos << "\n";
         
         //---
         // Read the first two bytes and test for SOC (Start Of Codestream)
         // marker.
         //---
         ossim_uint8 markerField[2];
         theFileStr->read( (char*)markerField, 2);

         bool foundSot = false;
         if ( (markerField[0] == 0xff) && (markerField[1] == 0x4f) )
         {
            // Get the SIZ marker and dump it.
            theFileStr->read( (char*)markerField, 2);
            if ( (markerField[0] == 0xff) && (markerField[1] == 0x51) )
            {
               ossimJ2kSizRecord siz;
               siz.parseStream( *theFileStr );
               siz.print(out);
            }
            
            // Find the firt tile marker.
            char c;
            while ( theFileStr->get(c) )
            {
               if (static_cast<ossim_uint8>(c) == 0xff)
               {
                  if ( theFileStr->get(c) )
                  {
                     out << "marker: 0xff" << hex << (ossim_uint16)c << dec << endl;
                     
                     if (static_cast<ossim_uint8>(c) == 0x52)
                     {
                        out << "\nFound COD...\n\n" << endl;
                        ossimJ2kCodRecord cod;
                        cod.parseStream( *theFileStr );
                        cod.print(out);

                     }
                     else if (static_cast<ossim_uint8>(c) == 0x55)
                     {
                        out << "\nFound TLM...\n\n" << endl;
                        ossimJ2kTlmRecord tlm;
                        tlm.parseStream( *theFileStr );
                        tlm.print(out);
                     }
                     else if (static_cast<ossim_uint8>(c) == 0x90)
                     {
                        foundSot = true;
                        break;
                     }
                  }
               }
            }
         }

         if (foundSot) // At SOT marker...
         {
            const ossim_uint32 BLOCKS =
               hdr->getNumberOfBlocksPerRow() * hdr->getNumberOfBlocksPerCol();
            for (ossim_uint32 i = 0; i < BLOCKS; ++i)
            {
               std::streamoff pos = theFileStr->tellg();
               out << "sot pos: " << pos << endl;
               ossimJ2kSotRecord sotRecord;
               sotRecord.parseStream( *theFileStr );
               pos += sotRecord.thePsot;
               sotRecord.print(out);
               theFileStr->seekg(pos, ios_base::beg);
            }

            // Find EOC marker
            char c;
            while ( theFileStr->get(c) )
            {
               if (static_cast<ossim_uint8>(c) == 0xff)
               {
                  if ( theFileStr->get(c) )
                  {
                     out << "marker: 0xff" << hex << (ossim_uint16)c << dec << endl;
                     
                     if (static_cast<ossim_uint8>(c) == 0xd9)
                     {
                        out << "EOC FOUND..." << endl;
                        out << "eoc pos: " << (theFileStr->tellg()-2) << endl;
		     }
		  }
	       }
	    }
         }
      }

      // If the last byte is read, the eofbit must be reset. 
      if ( theFileStr->eof() )
      {
         theFileStr->clear();
      }
      
      // Put the stream back to the where it was.
      theFileStr->seekg(currentPos);
   }

   return out;
}

bool ossimOpjNitfReader::uncompressJpegBlock(ossim_uint32 x,
                                             ossim_uint32 y)
{
   const ossimNitfImageHeader* hdr = getCurrentImageHeader();
   if (!hdr)
   {
      return false;
   }

   ossim_uint32 blockX = x / theCacheSize.x;
   ossim_uint32 blockY = y / theCacheSize.y;

   ossimIrect rect(ossimIpt(blockX * theCacheSize.x, blockY * theCacheSize.y),
		   ossimIpt((blockX + 1) * theCacheSize.x - 1, (blockY + 1) * theCacheSize.y - 1));

   std::streamoff offset = hdr->getDataLocation();
   theFileStr->seekg(offset, ios::beg);
   dumpTiles(ossimNotify(ossimNotifyLevel_WARN)); // KJJ
   ossim_int32 format = ossim::getCodecFormat(theFileStr.get());
   ossim::opj_decode(theFileStr.get(), rect, 0, format, offset, theCacheTile.get());

   return true;
}

