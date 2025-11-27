package us.ihmc.handsros2;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * A wrapper class that internally holds an array of YoDouble variables
 * and provides a convenient float-based API for accessing and modifying values.
 */
public class YoFloatArray
{
   private final YoDouble[] yoDoubles;

   /**
    * Creates a YoFloatArray with the specified size and initial values.
    *
    * @param namePrefix prefix for naming individual YoDouble variables
    * @param registry   YoRegistry to register the YoDouble variables
    * @param size       number of elements in the array
    * @param initialValues initial values for each element (varargs)
    */
   public YoFloatArray(String namePrefix, YoRegistry registry, float... initialValues)
   {
      this.yoDoubles = new YoDouble[initialValues.length];

      for (int i = 0; i < yoDoubles.length; i++)
      {
         yoDoubles[i] = new YoDouble(namePrefix + i, registry);
         yoDoubles[i].set(initialValues[i]);
      }
   }

   /**
    * Gets the float value at the specified index.
    *
    * @param index the index of the element to retrieve
    * @return the float value at the specified index
    */
   public float get(int index)
   {
      return (float) yoDoubles[index].getValue();
   }

   /**
    * Sets the float value at the specified index.
    *
    * @param index the index of the element to set
    * @param value the float value to set
    */
   public void set(int index, float value)
   {
      yoDoubles[index].set(value);
   }

   /**
    * Returns a copy of the internal data as a primitive float array.
    *
    * @return a new float array containing copies of all values
    */
   public float[] toFloatArray()
   {
      float[] result = new float[yoDoubles.length];
      for (int i = 0; i < yoDoubles.length; i++)
      {
         result[i] = (float) yoDoubles[i].getValue();
      }
      return result;
   }

   /**
    * Sets all values from a primitive float array.
    *
    * @param values the float array to copy values from
    */
   public void setAll(float[] values)
   {
      for (int i = 0; i < Math.min(yoDoubles.length, values.length); i++)
      {
         yoDoubles[i].set(values[i]);
      }
   }

   /**
    * Gets the underlying YoDouble at the specified index.
    * Useful for direct YoVariable operations.
    *
    * @param index the index of the YoDouble to retrieve
    * @return the YoDouble at the specified index
    */
   public YoDouble getYoDouble(int index)
   {
      return yoDoubles[index];
   }

   /**
    * Returns the size of this array.
    *
    * @return the number of elements in this array
    */
   public int size()
   {
      return yoDoubles.length;
   }

   /**
    * Returns a string representation of this YoFloatArray, listing all current values
    * in order with two decimal places of precision.
    *
    * @return a string in the form {@code YoFloatArray[30.00, 15.25, ...]}
    */
   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("YoFloatArray[");

      for (int i = 0; i < yoDoubles.length; i++)
      {
         if (i > 0)
            builder.append(", ");

         builder.append(String.format("%.2f", (float) yoDoubles[i].getValue()));
      }

      builder.append("]");
      return builder.toString();
   }
}
