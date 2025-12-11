package us.ihmc.handsros2;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * A wrapper class that internally holds an array of YoInteger variables
 * and provides a convenient int-based API for accessing and modifying values.
 */
public class YoIntegerArray
{
   private final YoInteger[] yoIntegers;
   private transient final int[] temp;

   /**
    * Creates a YoIntegerArray with the specified initial values.
    * The length of the array is determined by {@code initialValues.length}.
    *
    * @param namePrefix    prefix for naming individual YoInteger variables
    * @param registry      YoRegistry to register the YoInteger variables
    * @param initialValues initial values for each element (varargs)
    */
   public YoIntegerArray(String namePrefix, YoRegistry registry, int... initialValues)
   {
      this.yoIntegers = new YoInteger[initialValues.length];

      for (int i = 0; i < yoIntegers.length; i++)
      {
         yoIntegers[i] = new YoInteger(namePrefix + i, registry);
         yoIntegers[i].set(initialValues[i]);
      }

      temp = new int[yoIntegers.length];
   }

   /**
    * Gets the int value at the specified index.
    *
    * @param index the index of the element to retrieve
    * @return the int value at the specified index
    */
   public int get(int index)
   {
      return yoIntegers[index].getIntegerValue();
   }

   /**
    * Sets the int value at the specified index.
    *
    * @param index the index of the element to set
    * @param value the int value to set
    */
   public void set(int index, int value)
   {
      yoIntegers[index].set(value);
   }

   /**
    * Returns an internally kept array packed with updated primitive int values.
    * Make a copy if you need to hold onto this array.
    *
    * @return a reused int array containing copies of all values
    */
   public int[] toIntArray()
   {
      for (int i = 0; i < yoIntegers.length; i++)
      {
         temp[i] = yoIntegers[i].getIntegerValue();
      }
      return temp;
   }

   /**
    * Sets all values from a primitive int array.
    *
    * @param values the int array to copy values from
    */
   public void setAll(int[] values)
   {
      for (int i = 0; i < Math.min(yoIntegers.length, values.length); i++)
      {
         yoIntegers[i].set(values[i]);
      }
   }

   /**
    * Gets the underlying YoInteger at the specified index.
    * Useful for direct YoVariable operations.
    *
    * @param index the index of the YoInteger to retrieve
    * @return the YoInteger at the specified index
    */
   public YoInteger getYoInteger(int index)
   {
      return yoIntegers[index];
   }

   /**
    * Returns the size of this array.
    *
    * @return the number of elements in this array
    */
   public int size()
   {
      return yoIntegers.length;
   }

   /**
    * Returns a string representation of this YoIntegerArray, listing all current values
    * in order.
    *
    * @return a string in the form {@code YoIntegerArray[1, 2, 3, ...]}
    */
   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("YoIntegerArray[");

      for (int i = 0; i < yoIntegers.length; i++)
      {
         if (i > 0)
            builder.append(", ");

         builder.append(yoIntegers[i].getIntegerValue());
      }

      builder.append("]");
      return builder.toString();
   }
}
